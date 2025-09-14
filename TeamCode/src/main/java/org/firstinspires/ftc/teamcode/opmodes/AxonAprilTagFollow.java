package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "Axon: Follow AprilTag (Limelight3A)", group = "Vision")
public class AxonAprilTagFollow extends LinearOpMode {

    // Hardware names expected in RC configuration
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String AXON_SERVO_NAME = "axon"; // Factory Axon default logical name (change if needed)

    // Tunables
    private static final int    PIPELINE_INDEX = 0;   // Set this to your AprilTag pipeline index on Limelight
    private static final double KP_PAN         = 0.010; // Servo position per degree of X error (reduced to avoid overshoot)
    private static final double KD_PAN         = 0.000; // Disable derivative to avoid micro-chatter on noise
    private static final double SERVO_MIN      = 0.05; // Safety bounds for servo
    private static final double SERVO_MAX      = 0.95;
    private static final double DEAD_BAND_DEG  = 1.0;  // Wider deadband to ignore tiny jitter
    private static final double MAX_STEP       = 0.012; // Max change in servo per loop to prevent whipping
    private static final double SERVO_MIN_STEP = 0.002; // Ignore micro moves below this
    private static final boolean INVERT_PAN    = false; // Flip direction if your mount requires it
    // Filtering and hold behavior to prevent oscillation
    private static final double FILTER_ALPHA   = 0.50; // Heavier EMA smoothing for X error
    private static final double HOLD_ZONE_DEG  = 1.5;  // Inside this, freeze servo to avoid hunting
    private static final double EXIT_HOLD_DEG  = 3.0;  // Leave hold only if error grows beyond this (hysteresis)
    private static final int    HOLD_CONFIRM_FRAMES = 4; // frames required in hold before freezing
    private static final int    LOST_FRAMES_BEFORE_SWEEP = 10; // frames w/o tag before starting sweep
    private static final double SWEEP_STEP     = 0.005; // pan step while sweeping to reacquire

    private Limelight3A limelight;
    private Servo axon;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        axon = hardwareMap.get(Servo.class, AXON_SERVO_NAME);

        // Center servo at start
        double servoPos = 0.5;
        axon.setPosition(servoPos);

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
        boolean invert = INVERT_PAN;
        boolean xPressed = false;

        // Filter / derivative state
        boolean filterInit = false;
        double filteredX = 0.0;
        double prevFilteredX = 0.0;
        int holdFrames = 0;
        boolean inHold = false;

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL FPS", (int) status.getFps());
            telemetry.addData("LL Pipe", status.getPipelineIndex());

            // Runtime invert toggle: press X to flip direction if moving the wrong way
            if (gamepad1.x && !xPressed) { invert = !invert; xPressed = true; }
            if (!gamepad1.x) { xPressed = false; }

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

                // Hysteresis hold logic to avoid hunting near center
                if (!inHold) {
                    if (absFilt <= HOLD_ZONE_DEG) {
                        holdFrames++;
                        if (holdFrames >= HOLD_CONFIRM_FRAMES) {
                            inHold = true;
                        }
                    } else {
                        holdFrames = 0;
                    }
                } else {
                    // Stay in hold until error grows beyond exit threshold
                    if (absFilt >= EXIT_HOLD_DEG) {
                        inHold = false;
                        holdFrames = 0;
                    }
                }

                if (!inHold && absFilt > DEAD_BAND_DEG) {
                    double sign = invert ? 1.0 : -1.0;
                    double dErr = filteredX - prevFilteredX;
                    prevFilteredX = filteredX;

                    // PD control: proportional plus derivative damping
                    double cmd = KP_PAN * filteredX + KD_PAN * dErr;
                    double delta = sign * cmd;

                    // Limit the per-loop step to avoid sudden full rotations
                    if (delta >  MAX_STEP) delta =  MAX_STEP;
                    if (delta < -MAX_STEP) delta = -MAX_STEP;

                    // Ignore very small moves to keep servo fully steady
                    if (Math.abs(delta) < SERVO_MIN_STEP) {
                        // Skip commanding the servo to avoid tiny oscillations
                    } else {
                        servoPos = clamp(servoPos + delta, SERVO_MIN, SERVO_MAX);
                        axon.setPosition(servoPos);
                    }
                }
            } else {
                // No tag: after a few frames, sweep slowly to reacquire
                lostFrames++;
                if (lostFrames >= LOST_FRAMES_BEFORE_SWEEP) {
                    servoPos += sweepDir * SWEEP_STEP;
                    if (servoPos >= SERVO_MAX) { servoPos = SERVO_MAX; sweepDir = -1.0; }
                    if (servoPos <= SERVO_MIN) { servoPos = SERVO_MIN; sweepDir = 1.0; }
                    axon.setPosition(servoPos);
                }
                // Reset filter/hold when tag is lost
                filterInit = false;
                inHold = false;
                holdFrames = 0;
            }

            telemetry.addData("ServoPos", String.format("%.3f", servoPos));
            telemetry.addData("HaveTag", haveTag);
            telemetry.addData("Invert", invert);
            telemetry.addData("Xdeg", String.format("%.2f", xErrorDeg));
            telemetry.addData("XdegFilt", String.format("%.2f", filteredX));
            telemetry.addData("Hold", inHold);
            telemetry.update();

            // Small delay to reduce command spam
            sleep(10);
        }

        limelight.stop();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}


