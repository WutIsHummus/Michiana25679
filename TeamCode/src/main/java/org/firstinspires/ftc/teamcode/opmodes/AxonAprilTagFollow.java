package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "Axon: Follow AprilTag (Limelight3A)", group = "Vision")
public class AxonAprilTagFollow extends LinearOpMode {

    // Hardware names expected in RC configuration
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String AXON_SERVO_NAME = "axon"; // Logical name of the pan servo

    // Tunables
    private static final int    PIPELINE_INDEX = 0;   // Set this to your AprilTag pipeline index on Limelight
    private static final double KP_PAN         = 0.008; // Servo position change per degree of X error
    private static final double DEAD_BAND_DEG  = 1.5;   // Wider deadband to sit still more often
    private static final double SERVO_CENTER   = 0.5;   // Center position of servo (typically 0.5)
    private static final double SERVO_RANGE    = 0.4;   // Max range from center (+/- 0.4 gives 0.1 to 0.9)
    private static final double MAX_ERROR_DEG  = 30.0;  // Max expected X error for scaling
	// Search/Lock behavior
	private static final int    LOCK_FRAMES    = 3;     // consecutive frames inside lock window to acquire lock
	private static final double LOCK_WINDOW_DEG= 5.0;   // must be within this error to acquire lock
	private static final int    UNLOCK_LOST_FRAMES = 5; // frames without tag to drop lock
	private static final double UNLOCK_ERROR_DEG   = 12.0; // jump in error that immediately drops lock
	// Estimation when tag is briefly lost
	private static final double EST_POS_DECAY_PER_SEC = 1.0; // exponential-like decay of estimated position
	private static final double EST_VEL_DECAY_PER_SEC = 3.0; // exponential-like decay of estimated velocity
	private static final double EST_X_MAX_DEG        = 30.0; // cap estimate range (deg)
	private static final double EST_KEEP_LOCK_SEC    = 0.6;  // keep LOCK using estimate up to this long
	private static final double EST_SEARCH_USE_SEC   = 1.2;  // in SEARCH, prefer estimate before sweeping
    private static final boolean INVERT_PAN    = false; // Flip direction if your mount requires it
    // Filtering and hold behavior to prevent oscillation
    private static final double FILTER_ALPHA   = 0.25; // More smoothing to avoid twitch
    private static final int    LOST_FRAMES_BEFORE_SWEEP = 10; // frames w/o tag before starting sweep
    private static final double SWEEP_RANGE    = 0.05;  // How much to sweep from current position
    private static final double SWEEP_INTERVAL_SEC = 1.2; // flip a bit less frequently

    private Limelight3A limelight;
    private Servo axonServo;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        axonServo = hardwareMap.get(Servo.class, AXON_SERVO_NAME);
        try { imu = hardwareMap.get(IMU.class, "imu"); } catch (Throwable ignored) { imu = null; }

        // Configure servo - start at center
        axonServo.setPosition(SERVO_CENTER);
        double servoPosition = SERVO_CENTER;

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

        // Filter state
        boolean filterInit = false;
        double filteredX = 0.0;
	// Lock/search state
	int seenConsecutive = 0;
	int lostConsecutive = 0;
	boolean inLock = false;
	// Estimation state
	double estXDeg = 0.0;
	double estXVelDegPerSec = 0.0;
	double sinceLastValidSec = 0.0;
        long prevTimeNanos = System.nanoTime();
        // IMU tracking
        boolean imuInit = false;
        double prevYawDeg = 0.0;

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

            // Read IMU yaw delta
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

            // Simple proportional control to center X error by positioning servo
            if (haveTag) {
                lostFrames = 0;
                sinceLastValidSec = 0.0;

                // Initialize filter on first detection
                if (!filterInit) {
                    filteredX = xErrorDeg;
                    filterInit = true;
                } else {
                    // EMA filter to reduce noise
                    filteredX = FILTER_ALPHA * xErrorDeg + (1.0 - FILTER_ALPHA) * filteredX;
                }

                double absFilt = Math.abs(filteredX);
                // Update estimator from measurement
                estXDeg = filteredX;

		// Track lock acquisition
		if (!inLock) {
			if (absFilt <= LOCK_WINDOW_DEG) {
				seenConsecutive++;
				if (seenConsecutive >= LOCK_FRAMES) {
					inLock = true;
				}
			} else {
				seenConsecutive = 0;
			}
		}

		if (inLock) {
			// Proportional control while locked
			double sign = invert ? -1.0 : 1.0;
			// Use measurement if available, otherwise use estimate
			double xForControl = (sinceLastValidSec == 0.0) ? filteredX : estXDeg;

			// Apply or stop near center
			if (Math.abs(filteredX) <= DEAD_BAND_DEG) {
				// Within deadband, maintain current position
			} else {
				// Compute target servo position based on error
				double positionOffset = sign * KP_PAN * xForControl;
				positionOffset = clamp(positionOffset, -SERVO_RANGE, SERVO_RANGE);
				servoPosition = SERVO_CENTER + positionOffset;
				servoPosition = clamp(servoPosition, 0.0, 1.0);
				axonServo.setPosition(servoPosition);
			}

			// Unlock on large error spike
			if (Math.abs(filteredX) >= UNLOCK_ERROR_DEG) {
				inLock = false;
				seenConsecutive = 0;
			}
		} else {
			// SEARCH: move toward error if tag seen
			double sign = invert ? -1.0 : 1.0;
			double positionOffset = sign * KP_PAN * filteredX;
			positionOffset = clamp(positionOffset, -SERVO_RANGE, SERVO_RANGE);
			servoPosition = SERVO_CENTER + positionOffset;
			servoPosition = clamp(servoPosition, 0.0, 1.0);
			axonServo.setPosition(servoPosition);
		}
            } else {
                // No tag: after a few frames, sweep slowly to reacquire
                lostFrames++;
                lostConsecutive++;
                seenConsecutive = 0;
                if (lostConsecutive >= UNLOCK_LOST_FRAMES) inLock = false;
                // Propagate estimate using IMU yaw, then decay and clamp
                sinceLastValidSec += dt;
                estXDeg = estXDeg - deltaYawDeg;
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
                        double sign = invert ? -1.0 : 1.0;
                        double positionOffset = sign * KP_PAN * estXDeg;
                        positionOffset = clamp(positionOffset, -SERVO_RANGE, SERVO_RANGE);
                        servoPosition = SERVO_CENTER + positionOffset;
                        servoPosition = clamp(servoPosition, 0.0, 1.0);
                        axonServo.setPosition(servoPosition);
                    } else {
                        // Sweep back and forth
                        double sweepOffset = sweepDir * SWEEP_RANGE;
                        servoPosition = SERVO_CENTER + sweepOffset;
                        servoPosition = clamp(servoPosition, 0.0, 1.0);
                        axonServo.setPosition(servoPosition);
                    }
                }
                // Reset filter state when tag is lost
                filterInit = false;
            }

            telemetry.addData("ServoPos", String.format("%.3f", servoPosition));
            telemetry.addData("HaveTag", haveTag);
            telemetry.addData("Invert", invert);
            telemetry.addData("Xdeg", String.format("%.2f", xErrorDeg));
            telemetry.addData("XdegFilt", String.format("%.2f", filteredX));
            telemetry.addData("State", inLock ? "LOCK" : "SEARCH");
            telemetry.addData("SeenFrames", seenConsecutive);
            telemetry.addData("LostFrames", lostConsecutive);
            if (imuInit) telemetry.addData("YawDeg", String.format("%.1f", yawDeg));
            telemetry.update();

            // Small delay to reduce command spam
            sleep(10);
        }

        limelight.stop();
        if (axonServo != null) axonServo.setPosition(SERVO_CENTER);
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


