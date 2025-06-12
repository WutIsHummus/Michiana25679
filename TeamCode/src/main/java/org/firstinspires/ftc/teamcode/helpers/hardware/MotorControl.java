package org.firstinspires.ftc.teamcode.helpers.hardware;

import android.graphics.Color;
// PointF is not directly used for DetectorResult corners anymore,
// but might be used elsewhere or by other Limelight result types if you extend this.
// For now, we'll rely on List<List<Double>> for corners from DetectorResult.
// import android.graphics.PointF;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.data.Enums.DetectedColor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class MotorControl {

    public static PIDController liftController, extendoController;

    public final Servo ptor, ptol, led, lede, sweeper;
    public final Servo intakePivot, outtakeClaw, outtakeArmR, outtakeArmL;
    public final Servo outtakeLinkage, intakeArmR, intakeArmL;
    public final Servo hangr, hangl;

    public final RevColorSensorV3 specsensor;


    private float lastNormalizedRed = 0;
    private float lastNormalizedGreen = 0;
    private float lastNormalizedBlue = 0;

    public final RevColorSensorV3  colorSensor;
    public final DcMotorEx spin;

    public final Lift lift;
    public final Extendo extendo;

    public MotorControl(@NonNull HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        extendo = new Extendo(hardwareMap);

        ptor           = hardwareMap.get(Servo.class, "ptor");
        ptol           = hardwareMap.get(Servo.class, "ptol");
        led            = hardwareMap.get(Servo.class, "led");
        lede           = hardwareMap.get(Servo.class, "lede");
        sweeper        = hardwareMap.get(Servo.class, "sweeper");
        intakePivot    = hardwareMap.get(Servo.class, "intakepivot");
        outtakeClaw    = hardwareMap.get(Servo.class, "outtakeclaw");
        outtakeArmR    = hardwareMap.get(Servo.class, "outaker");
        outtakeArmL    = hardwareMap.get(Servo.class, "outakel");
        outtakeLinkage = hardwareMap.get(Servo.class, "outakelinkage");
        intakeArmR     = hardwareMap.get(Servo.class, "intaker");
        intakeArmL     = hardwareMap.get(Servo.class, "intakel");

        hangr = hardwareMap.get(Servo.class, "hangr");
        hangl = hardwareMap.get(Servo.class, "hangl");

        outtakeArmR.setDirection(Servo.Direction.REVERSE);
        intakeArmR.setDirection(Servo.Direction.REVERSE);
        hangr.setDirection(Servo.Direction.REVERSE);

        spin = hardwareMap.get(DcMotorEx.class, "spin");
        spin.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        specsensor = hardwareMap.get(RevColorSensorV3.class, "specsensor");


        outtakeClaw.setPosition(0.11);
    }

    public void update() {
        lift.update();
        extendo.update();
    }

    private double getDistanceCm() {
        DistanceSensor d = (DistanceSensor) colorSensor;
        return d.getDistance(DistanceUnit.CM);
    }

    public double getSpecSensorDistanceCm() {
        DistanceSensor d = (DistanceSensor) specsensor;
        return d.getDistance(DistanceUnit.CM);
    }

    private double[] normalizedRGB() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        double sum = r + g + b;
        if (sum <= 0) return new double[]{0.0, 0.0, 0.0};
        return new double[]{ r / sum, g / sum, b / sum };
    }

    public boolean isEmpty(){
        double distanceCm = getDistanceCm();

        if (distanceCm < 6){
            return  false;
        }
        return  true;
    }

    public Enums.DetectedColor getDetectedColor() {
        double distanceCm = getDistanceCm();
        double[] norm = normalizedRGB();

        // Compute scaled values: D * normalized channels
        double scaledR = distanceCm * norm[0];
        double scaledG = distanceCm * norm[1];
        double scaledB = distanceCm * norm[2];



        Enums.DetectedColor detected = Enums.DetectedColor.UNKNOWN;

        // Red thresholds:
        //   1.0  < D*Rnorm < 1.3
        //   0.9  < D*Gnorm < 1.7
        //   0.676< D*Bnorm < 1.45
        if (scaledR > 0.97  &&
                scaledG > 0.9  && scaledG < 1.4 &&
                scaledB < 1.2) {
            detected = Enums.DetectedColor.RED;
        }
        // Yellow thresholds:
        //   0.8  < D*Rnorm < 1.25
        //   1.2  < D*Gnorm < 1.9
        //   0.4  < D*Bnorm < 1.2
        else if (scaledR < 0.97 &&
                scaledG > 1.2 && scaledG < 1.6  &&
                scaledB > 0.4  && scaledB < 1.2) {
            detected = Enums.DetectedColor.YELLOW;
        }
        // Blue thresholds:
        //   0.5  < D*Rnorm < 1.0
        //   0.95 < D*Gnorm < 1.9
        //   1.45 < D*Bnorm < 2.0
        else if (scaledR > 0.5   && scaledR < 1.05  &&
                scaledG > 0.95  && scaledG < 1.94  &&
                scaledB > 1.43 ) {
            detected = Enums.DetectedColor.BLUE;
        }

        if (distanceCm > 6){
            detected = Enums.DetectedColor.UNKNOWN;
        }
        return detected;
    }




    public abstract static class ControlledDevice {
        public CachingDcMotorEx motor;
        public boolean resetting = false;
        double targetPosition;

        public double getTargetPosition() {
            return targetPosition;
        }

        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        public boolean isResetting() {
            return resetting;
        }

        public abstract void update();
        public abstract void reset();
        public abstract boolean closeEnough();

        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }
    }

    public static class Extendo extends ControlledDevice {
        private static final double p = 0.005, i = 0, d = 0.0001;

        public Extendo(HardwareMap hardwareMap) {
            extendoController = new PIDController(p, i, d);
            motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendo"), 0.05);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            resetting = false;
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.7);
            resetting = true;
        }

        public void update() {
            if (resetting) {
                if (Math.abs(motor.getVelocity())  < 5) {
                    reset();
                    resetting = false;
                }
            } else {
                extendoController.setPID(p, i, d);
                double pid = extendoController.calculate(motor.getCurrentPosition(), targetPosition);
                motor.setPower(pid);
            }
        }

        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 20;
        }

        public boolean closeEnough(double target) {
            return Math.abs(motor.getCurrentPosition() - target) < 20;
        }


        public boolean closeEnough(double target, double range) {
            return Math.abs(motor.getCurrentPosition() - target) < range;
        }
    }

    public static class Lift extends ControlledDevice {
        public CachingDcMotorEx motor2;
        public static final double p = -0.02, i = 0, d = -0.0003;
        // public static final double GRAVITY_FEEDFORWARD = 0; // Not used in current update

        public Lift(HardwareMap hardwareMap) {
            liftController = new PIDController(p, i, d);
            motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftr"), 0.005);
            motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftl"), 0.005);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            // Assuming RUN_WITHOUT_ENCODER as per original Extendo, adjust if Lift uses encoders differently
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void reset() {
            motor.setPower(0);
            motor2.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // motor2 should also be reset if it's position-controlled or synced
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            resetting = false;
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.6);
            motor2.setPower(0.6);
            resetting = true;
        }

        public void update() {
            if (resetting) {
                if (Math.abs(motor.getVelocity()) < 5) { // Assuming primary motor velocity indicates stop
                    reset();
                    resetting = false;
                }
            } else {
                liftController.setPID(p, i, d);
                // Assuming motor1 is the primary for position feedback
                double pid = liftController.calculate(motor.getCurrentPosition(), targetPosition);
                double motorPower = pid;
                motor.setPower(motorPower);
                motor2.setPower(motorPower); // Sync power
            }
        }

        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 20;
        }

        public boolean closeEnough(double target) {
            return Math.abs(motor.getCurrentPosition() - target) < 20;
        }

        public boolean closeEnough(double target, double range) {
            return Math.abs(motor.getCurrentPosition() - target) < range;
        }
    }

    // REVAMPED Limelight Class
    public static class Limelight {
        private final Limelight3A limelight;
        private final Telemetry telemetry;

        final double IMAGE_WIDTH  = 2592;
        final double IMAGE_HEIGHT = 1944;

        final double CENTER_X = IMAGE_WIDTH  / 2.0;
        final double CENTER_Y = IMAGE_HEIGHT * 2 / 3.0;

        private static final int PIPE_A = 0;
        private static final int PIPE_B = 1;
        private String PRIMARY_CLASS = "blue";
        private static final Map<String, Double> COLOR_MAP = new HashMap<String, Double>() {{
            put("red", 0.0);
            put("blue", 1.0);
            put("yellow", 2.0);
        }};

        private final int maxSamples = 1;
        private final List<Double> horizontalMeasurements = new ArrayList<>();
        private final List<Double> forwardMeasurements = new ArrayList<>();
        private final List<Double> angleMeasurements = new ArrayList<>();
        private boolean isCollectingSamples;

        public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            telemetry.addData("Limelight", "Initialized and polling started.");
        }

        public void setPrimaryClass(String primaryClass) {
            this.PRIMARY_CLASS = primaryClass.toLowerCase();
        }

        public void stop() {
            limelight.stop();
        }

        public void resetSamples() {
            horizontalMeasurements.clear();
            forwardMeasurements.clear();
            angleMeasurements.clear();
            isCollectingSamples = false;
        }

        public boolean isCollectingSamples() {
            return isCollectingSamples;
        }

        public void startCollectingSamples() {
            resetSamples();
            isCollectingSamples = true;
        }



        // Corrected getBoundingBox to use List<List<Double>>
        private double[] getBoundingBox(List<List<Double>> corners) {
            if (corners == null || corners.isEmpty()) {
                return new double[]{0, 0, 0, 0}; // x, y, w, h
            }
            double minX = corners.get(0).get(0);
            double maxX = corners.get(0).get(0);
            double minY = corners.get(0).get(1);
            double maxY = corners.get(0).get(1);

            for (List<Double> p : corners) {
                if (p.size() == 2) {
                    minX = Math.min(minX, p.get(0));
                    maxX = Math.max(maxX, p.get(0));
                    minY = Math.min(minY, p.get(1));
                    maxY = Math.max(maxY, p.get(1));
                }
            }
            return new double[]{minX, minY, maxX - minX, maxY - minY}; // x, y, w, h
        }


        public Action collectSamplesAction() {
            return new Action() {
                private long startTime = -1;
                private long nextTime = -1;
                private boolean inPipelineA = true;
                private boolean processingB = false;

                @Override
                public boolean run(TelemetryPacket tp) {
                    long now = System.currentTimeMillis();

                    // Initialize collection on first run
                    if (startTime < 0) {
                        startTime = now;
                        startCollectingSamples();
                        limelight.pipelineSwitch(PIPE_A);
                        nextTime = now + 20;
                        return true;
                    }

                    // Abort if overall timeout exceeded (2 seconds)
                    if (now - startTime > 6000) {
                        isCollectingSamples = false;
                        return false;
                    }

                    // Stage A: process pipeline A result after ~20ms
                    if (inPipelineA) {
                        if (now < nextTime) {
                            return true; // still waiting
                        }
                        LLResult resultA = limelight.getLatestResult();
                        if (resultA == null || resultA.getDetectorResults() == null) {
                            nextTime = now + 20;
                            return true;
                        }
                        List<LLResultTypes.DetectorResult> detsA = resultA.getDetectorResults();
                        if (detsA.isEmpty()) {
                            nextTime = now + 20;
                            return true;
                        }
                        // Split detections into primary-class vs extras
                        List<LLResultTypes.DetectorResult> primaries = new ArrayList<>();
                        List<LLResultTypes.DetectorResult> extras = new ArrayList<>();
                        for (LLResultTypes.DetectorResult d : detsA) {
                            if (d.getClassName().equalsIgnoreCase(PRIMARY_CLASS)) {
                                primaries.add(d);
                            } else {
                                extras.add(d);
                            }
                        }
                        if (primaries.isEmpty()) {
                            nextTime = now + 20;
                            return true;
                        }
                        // Choose the largest primary target by area
                        LLResultTypes.DetectorResult primary = Collections.max(primaries, Comparator.comparingDouble(d -> {
                            double[] box = getBoundingBox(d.getTargetCorners());
                            return box[2] * box[3];
                        }));
                        for (LLResultTypes.DetectorResult d : primaries) {
                            if (d != primary) extras.add(d);
                        }
                        double[] pBox = getBoundingBox(primary.getTargetCorners());
                        double colorCode = COLOR_MAP.getOrDefault(primary.getClassName().toLowerCase(), 1.0);
                        // Compute subtraction ROIs from other detections
                        // Prepare inputs: primary box + color + each sub-ROI
                        ArrayList<Double> inputs = new ArrayList<>();
                        for (double v : pBox) inputs.add(v);
                        inputs.add(colorCode);

                        // Switch to pipeline B and send inputs to Python
                        limelight.pipelineSwitch(PIPE_B);
                        limelight.updatePythonInputs(inputs.stream().mapToDouble(Double::doubleValue).toArray());

                        // Move to pipeline B stage
                        inPipelineA = false;
                        processingB = true;
                        nextTime = now + 30;
                        return true;
                    }

                    // Stage B: process pipeline B result after ~30ms
                    if (processingB) {
                        if (now < nextTime) {
                            return true; // still waiting
                        }
                        LLResult resultB = limelight.getLatestResult();
                        processingB = false;
                        if (resultB != null && resultB.getPythonOutput() != null) {
                            double[] out = resultB.getPythonOutput();
                            if (out.length >= 3) {
                                double forward = out[0], horiz = out[1], ang = out[2];
                                forwardMeasurements.add(forward);
                                horizontalMeasurements.add(horiz);
                                angleMeasurements.add(ang);
                                // Keep only the latest maxSamples entries
                                if (forwardMeasurements.size() > maxSamples)
                                    forwardMeasurements.remove(0);
                                if (horizontalMeasurements.size() > maxSamples)
                                    horizontalMeasurements.remove(0);
                                if (angleMeasurements.size() > maxSamples)
                                    angleMeasurements.remove(0);
                                telemetry.addData("Limelight Sample",
                                        String.format("Fwd: %.1f, Horz: %.1f, Ang: %.1f", forward, horiz, ang));
                            }
                        }
                        // If we've collected enough samples, finish
                        if (forwardMeasurements.size() >= maxSamples) {
                            isCollectingSamples = false;
                            return false;
                        }
                        // Otherwise, go back to pipeline A for another round
                        inPipelineA = true;
                        limelight.pipelineSwitch(PIPE_A);
                        nextTime = now + 20;
                        return true;
                    }

                    // Should not reach here, but just in case
                    isCollectingSamples = false;
                    return false;
                }
            };
        }


        public Vector2d getAveragePose() {
            if (horizontalMeasurements.isEmpty() || forwardMeasurements.isEmpty()) {
                telemetry.addLine("Limelight Average Pose: No samples collected.");
                return new Vector2d(0, 0);
            }

            double avgHorizontal = horizontalMeasurements.stream().mapToDouble(Double::doubleValue).average().orElse(0);
            double avgForward = forwardMeasurements.stream().mapToDouble(Double::doubleValue).average().orElse(0);

            telemetry.addData("Limelight Avg Horizontal", String.format("%.2f", avgHorizontal));
            telemetry.addData("Limelight Avg Forward", String.format("%.2f", avgForward));
            return new Vector2d(avgHorizontal, avgForward);
        }

        public double getAverageAngle() {
            if (angleMeasurements.isEmpty()) {
                telemetry.addLine("Limelight Average Angle: No samples collected.");
                return 0;
            }
            double avgAngle = angleMeasurements.stream().mapToDouble(Double::doubleValue).average().orElse(0);
            telemetry.addData("Limelight Avg Angle", String.format("%.2f", avgAngle));
            return avgAngle;
        }
    }
}