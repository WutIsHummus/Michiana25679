package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "Axon: Follow AprilTag (Limelight3A)", group = "Vision")
public class AxonAprilTagFollow extends LinearOpMode {

    // Hardware names expected in RC configuration
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String AXON_MOTOR_NAME = "axon"; // Logical name of the pan motor

    // Tunables
    private static final int    PIPELINE_INDEX = 0;   // Set this to your AprilTag pipeline index on Limelight
    private static final double KP_PAN         = 0.010; // Motor power per degree of X error (slower)
    private static final double KD_PAN         = 0.006; // Motor power per (deg/sec) error (more damping)
    private static final double DEAD_BAND_DEG  = 1.5;   // Wider deadband to sit still more often
    private static final double POWER_MAX      = 0.35;  // Lower cap to slow motion
	// Search/Lock behavior
	private static final double SEARCH_POWER   = 0.005; // constant slow power while searching
	private static final int    LOCK_FRAMES    = 3;     // consecutive frames inside lock window to acquire lock
	private static final double LOCK_WINDOW_DEG= 5.0;   // must be within this error to acquire lock
	private static final int    UNLOCK_LOST_FRAMES = 5; // frames without tag to drop lock
	private static final double UNLOCK_ERROR_DEG   = 12.0; // jump in error that immediately drops lock
	private static final double AUTO_INVERT_CHECK_SEC = 0.25; // after locking, time to verify direction
	private static final double AUTO_INVERT_MIN_IMPROVE_DEG = 1.5; // require at least this improvement
	// Estimation when tag is briefly lost
	private static final double EST_POS_DECAY_PER_SEC = 1.0; // exponential-like decay of estimated position
	private static final double EST_VEL_DECAY_PER_SEC = 3.0; // exponential-like decay of estimated velocity
	private static final double EST_X_MAX_DEG        = 30.0; // cap estimate range (deg)
	private static final double EST_KEEP_LOCK_SEC    = 0.6;  // keep LOCK using estimate up to this long
	private static final double EST_SEARCH_USE_SEC   = 1.2;  // in SEARCH, prefer estimate before sweeping
	private static final double LOSS_P_SCALE         = 0.6;  // scale P while tracking on estimate
	private static final double LOSS_D_SCALE         = 0.5;  // scale D while tracking on estimate
    // Rate limits are time-normalized (per second); converted to per-loop using dt
    private static final double MAX_POWER_STEP_PER_SEC = 0.30;  // Tighter slew to slow accelerations
    private static final double MIN_POWER_STEP_PER_SEC = 0.08;  // Still ignore micro changes
    private static final boolean INVERT_PAN    = false; // Flip direction if your mount requires it
    // Filtering and hold behavior to prevent oscillation
    private static final double FILTER_ALPHA   = 0.25; // More smoothing to avoid twitch
    private static final double HOLD_ZONE_DEG  = 1.5;  // Inside this, freeze servo to avoid hunting
    private static final double EXIT_HOLD_DEG  = 3.0;  // Leave hold only if error grows beyond this (hysteresis)
    private static final int    HOLD_CONFIRM_FRAMES = 4; // frames required in hold before freezing
    private static final int    LOST_FRAMES_BEFORE_SWEEP = 10; // frames w/o tag before starting sweep
    private static final double SWEEP_POWER    = 0.08;  // slower sweep power
    private static final double SWEEP_INTERVAL_SEC = 1.2; // flip a bit less frequently
    // Pan encoder config (adjust to your hardware)
    private static final double PAN_TICKS_PER_REV = 537.6; // e.g., goBILDA 312 RPM motor encoder
    private static final double PAN_GEAR_RATIO    = 1.0;   // output shaft revs per motor rev
    private static final int    PAN_TICK_SIGN     = 1;     // flip to match positive pan direction

    private Limelight3A limelight;
    private DcMotorEx axonMotor;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        axonMotor = hardwareMap.get(DcMotorEx.class, AXON_MOTOR_NAME);
        try { imu = hardwareMap.get(IMU.class, "imu"); } catch (Throwable ignored) { imu = null; }

        // Configure motor
        axonMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        axonMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double motorPower = 0.0;
        axonMotor.setPower(0.0);

        // Configure Limelight
        try { limelight.pipelineSwitch(PIPELINE_INDEX); } catch (Throwable ignored) {}
        try { limelight.setPollRateHz(250); } catch (Throwable ignored) {}
        limelight.start();

        telemetry.addLine("> Axon pan active. Ensure Limelight AprilTag pipeline is selected.")
                .addData("Pipeline", PIPELINE_INDEX);
        telemetry.update();

        waitForStart();

        int lostFrames = 0;
        double sweepDir = 1.0;
        double sweepTimerSec = 0.0;
        boolean invert = INVERT_PAN;
        boolean xPressed = false;

        // Filter / derivative state
        boolean filterInit = false;
        double filteredX = 0.0;
        double prevFilteredX = 0.0;
	int holdFrames = 0;
	boolean inHold = false;
	// Lock/search state
	int seenConsecutive = 0;
	int lostConsecutive = 0;
	boolean inLock = false;
	double lockCheckTimerSec = 0.0;
	double lockInitialAbsError = 0.0;
	boolean pendingAutoInvertCheck = false;
	boolean invertFlippedThisLock = false;
	// Estimation state
	double estXDeg = 0.0;
	double estXVelDegPerSec = 0.0;
	double sinceLastValidSec = 0.0;
        long prevTimeNanos = System.nanoTime();
        // IMU and pan encoder tracking
        boolean imuInit = false;
        double prevYawDeg = 0.0;
        int prevPanTicks = axonMotor.getCurrentPosition();
        final double degPerTick = 360.0 / (PAN_TICKS_PER_REV * PAN_GEAR_RATIO);

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL FPS", (int) status.getFps());
            telemetry.addData("LL Pipe", status.getPipelineIndex());

            // Runtime invert toggle: press X to flip direction if moving the wrong way
            if (gamepad1.x && !xPressed) { invert = !invert; xPressed = true; }
            if (!gamepad1.x) { xPressed = false; }

            // Compute loop delta time (seconds) for time-normalized damping and rate limits
            long now = System.nanoTime();
            double dt = (now - prevTimeNanos) / 1e9;
            // Clamp dt to a reasonable range to avoid spikes on first frame or pauses
            if (dt <= 0 || dt > 0.1) dt = 0.01;
            prevTimeNanos = now;

            // Read IMU yaw and pan encoder deltas
            double yawDeg = 0.0;
            double deltaYawDeg = 0.0;
            if (imu != null) {
                try {
                    yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    if (!imuInit) { prevYawDeg = yawDeg; imuInit = true; }
                    deltaYawDeg = wrapDeltaDeg(yawDeg - prevYawDeg);
                    prevYawDeg = yawDeg;
                } catch (Throwable ignored) {}
            }
            int panTicks = axonMotor.getCurrentPosition();
            int deltaPanTicks = panTicks - prevPanTicks;
            prevPanTicks = panTicks;
            double deltaPanDeg = PAN_TICK_SIGN * deltaPanTicks * degPerTick;

            LLResult result = limelight.getLatestResult();
            double xErrorDeg = 0.0;
            boolean haveTag = false;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    // Choose the largest area tag (closest/most centered)
                    LLResultTypes.FiducialResult best = null;
                    double bestArea = -1.0;
                    for (LLResultTypes.FiducialResult fr : tags) {
                        double area = fr.getTargetArea();
                        if (area > bestArea) { bestArea = area; best = fr; }
                    }
                    if (best != null) {
                        xErrorDeg = best.getTargetXDegrees();
                        haveTag = true;
                        telemetry.addData("TagID", best.getFiducialId());
                        telemetry.addData("Xdeg", String.format("%.2f", xErrorDeg));
                        telemetry.addData("Ydeg", String.format("%.2f", best.getTargetYDegrees()));
                        telemetry.addData("Area", String.format("%.3f", bestArea));
                    }
                }
            }

            // Simple P control to center X error by panning servo
            if (haveTag) {
                lostFrames = 0;
                sinceLastValidSec = 0.0;

                // Initialize filter on first detection
                if (!filterInit) {
                    filteredX = xErrorDeg;
                    prevFilteredX = filteredX;
                    filterInit = true;
                    inHold = false;
                    holdFrames = 0;
                } else {
                    // EMA filter to reduce noise
                    filteredX = FILTER_ALPHA * xErrorDeg + (1.0 - FILTER_ALPHA) * filteredX;
                }

                double absFilt = Math.abs(filteredX);
                // Update estimator from measurement
                double instantVel = (filteredX - prevFilteredX) / dt;
                estXDeg = filteredX;
                estXVelDegPerSec = instantVel;

		// Track lock acquisition
		if (!inLock) {
			if (absFilt <= LOCK_WINDOW_DEG) {
				seenConsecutive++;
				if (seenConsecutive >= LOCK_FRAMES) {
					inLock = true;
					// prepare auto-invert check
					lockInitialAbsError = absFilt;
					lockCheckTimerSec = 0.0;
					pendingAutoInvertCheck = true;
					invertFlippedThisLock = false;
				}
			} else {
				seenConsecutive = 0;
			}
		}

		if (inLock) {
			// Auto-invert check shortly after lock to ensure correct direction
			lockCheckTimerSec += dt;
			if (pendingAutoInvertCheck && lockCheckTimerSec >= AUTO_INVERT_CHECK_SEC) {
				if (Math.abs(filteredX) > lockInitialAbsError - AUTO_INVERT_MIN_IMPROVE_DEG && !invertFlippedThisLock) {
					invert = !invert;
					invertFlippedThisLock = true;
				}
				pendingAutoInvertCheck = false;
			}

			// PD tracking while locked
			double sign = invert ? 1.0 : -1.0;
			// Use measurement if available, otherwise use estimate for PD
			double xForControl = filteredX;
			double dErrPerSec;
			if (sinceLastValidSec <= EST_KEEP_LOCK_SEC) {
				// When tag briefly lost but still in lock window: use estimate with scaled gains
				xForControl = (sinceLastValidSec == 0.0) ? filteredX : estXDeg;
				dErrPerSec = (sinceLastValidSec == 0.0) ? (filteredX - prevFilteredX) / dt : estXVelDegPerSec;
			} else {
				// Normal case: measured
				dErrPerSec = (filteredX - prevFilteredX) / dt;
			}
			prevFilteredX = filteredX;

			// PD control to compute target power
			double kp = (sinceLastValidSec == 0.0) ? KP_PAN : KP_PAN * LOSS_P_SCALE;
			double kd = (sinceLastValidSec == 0.0) ? KD_PAN : KD_PAN * LOSS_D_SCALE;
			double targetPower = sign * (kp * xForControl + kd * dErrPerSec);
			// Clamp target power
			if (targetPower >  POWER_MAX) targetPower =  POWER_MAX;
			if (targetPower < -POWER_MAX) targetPower = -POWER_MAX;

			// Time-normalized power rate limiting
			double maxStep = MAX_POWER_STEP_PER_SEC * dt;
			double minStep = MIN_POWER_STEP_PER_SEC * dt;
			double delta = targetPower - motorPower;
			if (delta >  maxStep) delta =  maxStep;
			if (delta < -maxStep) delta = -maxStep;

			// Apply or stop near center
			if (Math.abs(filteredX) <= DEAD_BAND_DEG) {
				if (motorPower != 0.0) {
					motorPower = 0.0;
					axonMotor.setPower(0.0);
				}
			} else if (Math.abs(delta) < minStep) {
				// too small to change, hold current
			} else {
				motorPower += delta;
				if (motorPower >  POWER_MAX) motorPower =  POWER_MAX;
				if (motorPower < -POWER_MAX) motorPower = -POWER_MAX;
				axonMotor.setPower(motorPower);
			}

			// Unlock on large error spike
			if (Math.abs(filteredX) >= UNLOCK_ERROR_DEG) {
				inLock = false;
				seenConsecutive = 0;
			}
		} else {
			// SEARCH: move slowly at constant power, biased to reduce error if tag seen
			double dir = (result != null && result.isValid()) ? Math.signum(filteredX) : (sweepDir);
			double sign = invert ? 1.0 : -1.0;
			motorPower = SEARCH_POWER * dir * sign;
			axonMotor.setPower(motorPower);
		}
            } else {
                // No tag: after a few frames, sweep slowly to reacquire
                lostFrames++;
                lostConsecutive++;
                seenConsecutive = 0;
                if (lostConsecutive >= UNLOCK_LOST_FRAMES) inLock = false;
                // Propagate estimate using IMU yaw and pan encoder, then decay and clamp
                sinceLastValidSec += dt;
                estXDeg = estXDeg - deltaPanDeg - deltaYawDeg;
                estXVelDegPerSec = -(deltaPanDeg + deltaYawDeg) / Math.max(dt, 1e-3);
                estXVelDegPerSec *= Math.max(0.0, 1.0 - EST_VEL_DECAY_PER_SEC * dt);
                estXDeg *= Math.max(0.0, 1.0 - EST_POS_DECAY_PER_SEC * dt);
                if (estXDeg >  EST_X_MAX_DEG) estXDeg =  EST_X_MAX_DEG;
                if (estXDeg < -EST_X_MAX_DEG) estXDeg = -EST_X_MAX_DEG;
                if (lostFrames >= LOST_FRAMES_BEFORE_SWEEP) {
                    sweepTimerSec += dt;
                    if (sweepTimerSec >= SWEEP_INTERVAL_SEC) {
                        sweepDir *= -1.0;
                        sweepTimerSec = 0.0;
                    }
                    // If we still have a fresh estimate, prefer driving toward it instead of sweeping
                    if (sinceLastValidSec <= EST_SEARCH_USE_SEC) {
                        double dir = Math.signum(estXDeg);
                        double sign = invert ? 1.0 : -1.0;
                        motorPower = SEARCH_POWER * dir * sign;
                        axonMotor.setPower(motorPower);
                    } else {
                        double sweepPower = sweepDir * SWEEP_POWER * (invert ? 1.0 : -1.0);
                        motorPower = sweepPower;
                        axonMotor.setPower(SEARCH_POWER);
                    }
                }
                // Reset filter state when tag is lost
                filterInit = false;
            }

            telemetry.addData("MotorPower", String.format("%.3f", motorPower));
            telemetry.addData("HaveTag", haveTag);
            telemetry.addData("Invert", invert);
            telemetry.addData("Xdeg", String.format("%.2f", xErrorDeg));
            telemetry.addData("XdegFilt", String.format("%.2f", filteredX));
            telemetry.addData("State", inLock ? "LOCK" : "SEARCH");
            telemetry.addData("SeenFrames", seenConsecutive);
            telemetry.addData("LostFrames", lostConsecutive);
            if (imuInit) telemetry.addData("YawDeg", String.format("%.1f", yawDeg));
            telemetry.addData("PanTicks", panTicks);
            telemetry.update();

            // Small delay to reduce command spam
            sleep(10);
        }

        limelight.stop();
        if (axonMotor != null) axonMotor.setPower(0.0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapDeltaDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }
}


