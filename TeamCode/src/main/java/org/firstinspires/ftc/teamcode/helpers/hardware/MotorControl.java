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
import java.util.Objects;
import java.util.Set;
import java.util.stream.Collectors;

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

        if (scaledR > 0.8  &&
                scaledG < 0.99  &&
                scaledB < 0.7) {
            detected = Enums.DetectedColor.RED;
        }
        else if (scaledR < 0.78 &&
                scaledG > 0.9 &&
                scaledB < 0.5) {
            detected = Enums.DetectedColor.YELLOW;
        }
        else if (scaledB > 1.15 ) {
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
            motor.setPower(-0.9);
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

    public static class Limelight {
        private final Limelight3A limelight;
        private final Telemetry telemetry;

        private static final double CAMERA_HEIGHT_M = 0.61;
        private static final double CAMERA_PITCH_DEG = 29.0;
        private static final double IMAGE_WIDTH_PX = 2592.0;
        private static final double IMAGE_HEIGHT_PX = 1944.0;
        private static final double VERTICAL_FOV_DEG = 42.0;
        private static final double FOCAL_LENGTH_PX = (IMAGE_HEIGHT_PX / 2.0)
                / Math.tan(Math.toRadians(VERTICAL_FOV_DEG / 2.0));
        public double offset = 4;
        private boolean upsideDown = true;

        private List<String> targetClasses = new ArrayList<>(List.of("blue"));


        private final List<DetectionResult> cachedDetections = new ArrayList<>();
        private int currentSampleIndex = 0;

        public Limelight(HardwareMap hardwareMap, Telemetry telemetry,double offset) {
            this.telemetry = telemetry;
            this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.start();
            this.offset = offset;
            telemetry.addData("Limelight", "Initialized and polling started.");
        }

        public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.start();
            limelight.setPollRateHz(250);
            telemetry.addData("Limelight", "Initialized and polling started.");
        }

        public void setUpsideDown(boolean upsideDown) {
            this.upsideDown = upsideDown;
        }

        public void setTargetClasses(String... classes) {
            targetClasses.clear();
            for (String cls : classes) {
                targetClasses.add(cls.toLowerCase());
            }
        }

        public void stop() {
            limelight.stop();
        }

        public DetectionResult getDistance(double distanceCost) {
            LLResult result = limelight.getLatestResult();
            if (result == null || result.getDetectorResults() == null
                    || result.getDetectorResults().isEmpty()) {
                telemetry.addLine("No detections found.");
                cachedDetections.clear();
                return null;
            }

            List<LLResultTypes.DetectorResult> matches = result.getDetectorResults().stream()
                    .filter(d -> targetClasses.stream()
                            .anyMatch(tc -> tc.equalsIgnoreCase(d.getClassName())))
                    .collect(Collectors.toList());

            if (matches.isEmpty()) {
                telemetry.addLine("Primary class not detected.");
                cachedDetections.clear();
                return null;
            }

            cachedDetections.clear();
            for (LLResultTypes.DetectorResult det : matches) {
                double[] box = getBoundingBox(det.getTargetCorners());
                double bottomX = box[0] + box[2] * 0.5;
                double bottomY = box[1] + box[3];

                double rawDx = bottomX - (IMAGE_WIDTH_PX  / 2.0);
                double rawDy = bottomY - (IMAGE_HEIGHT_PX / 2.0);

                // if upsideDown, flip both axes
                double dx = upsideDown ? -rawDx : rawDx;
                double dy = upsideDown ? -rawDy : rawDy;

                double yawRad = Math.atan(dx / FOCAL_LENGTH_PX);
                double pitchRad = Math.atan(dy / FOCAL_LENGTH_PX);
                double totalPitch = Math.toRadians(CAMERA_PITCH_DEG) + pitchRad;

                double distM = CAMERA_HEIGHT_M / Math.tan(totalPitch);
                double distIn = distM * 39.3701 - 12;
                double yawDeg = Math.toDegrees(yawRad) + 2;

                double cost = Math.abs(Math.abs(yawDeg) - offset) + distM * distanceCost;
                cachedDetections.add(new DetectionResult(distIn, yawDeg, cost));
            }

            cachedDetections.sort(Comparator.comparingDouble(d -> d.cost));
            currentSampleIndex = 0;

            if (!cachedDetections.isEmpty()) {
                DetectionResult best = cachedDetections.get(0);
                telemetry.addData("Best Dist (in)", String.format("%.1f", best.distanceInches));
                telemetry.addData("Best Yaw (°)", String.format("%.1f", best.yawDegrees));
                return best;
            }

            return null;
        }

        public DetectionResult getNextSample() {
            currentSampleIndex++;
            if (currentSampleIndex >= cachedDetections.size()) {
                telemetry.addLine("No more samples available.");
                return null;
            }

            DetectionResult next = cachedDetections.get(currentSampleIndex);
            telemetry.addData("Next Dist (in)", String.format("%.1f", next.distanceInches));
            telemetry.addData("Next Yaw (°)", String.format("%.1f", next.yawDegrees));
            return next;
        }

        private double[] getBoundingBox(List<List<Double>> corners) {
            double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE, maxY = Double.MIN_VALUE;
            for (List<Double> p : corners) {
                minX = Math.min(minX, p.get(0));
                maxX = Math.max(maxX, p.get(0));
                minY = Math.min(minY, p.get(1));
                maxY = Math.max(maxY, p.get(1));
            }
            return new double[]{minX, minY, maxX - minX, maxY - minY};
        }

        public static class DetectionResult {
            public final double distanceInches;
            public final double yawDegrees;
            public final double cost;

            public DetectionResult(double distanceInches, double yawDegrees, double cost) {
                this.distanceInches = distanceInches;
                this.yawDegrees = yawDegrees;
                this.cost = cost;
            }

            @Override
            public boolean equals(Object o) {
                if (!(o instanceof DetectionResult)) return false;
                DetectionResult other = (DetectionResult) o;
                return Math.abs(this.distanceInches - other.distanceInches) < 0.5 &&
                        Math.abs(this.yawDegrees - other.yawDegrees) < 0.5;
            }

            @Override
            public int hashCode() {
                return Objects.hash((int)(distanceInches * 2), (int)(yawDegrees * 2));
            }
        }
    }

}