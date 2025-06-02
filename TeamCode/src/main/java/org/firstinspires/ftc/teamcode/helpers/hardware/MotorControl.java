package org.firstinspires.ftc.teamcode.helpers.hardware;

import android.graphics.Color;
// PointF is not directly used for DetectorResult corners anymore,
// but might be used elsewhere or by other Limelight result types if you extend this.
// For now, we'll rely on List<List<Double>> for corners from DetectorResult.
// import android.graphics.PointF;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public final ColorSensor colorSensor;
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
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        colorSensor.enableLed(false);
    }

    public void update() {
        lift.update();
        extendo.update();
    }

    public DetectedColor getDetectedColor() {
        float red = colorSensor.red();
        float green = colorSensor.green();
        float blue = colorSensor.blue();

        float maxRawValue = Math.max(red, Math.max(green, blue));
        if (maxRawValue == 0) return DetectedColor.UNKNOWN; // Avoid division by zero
        float scale = 255 / maxRawValue;

        red *= scale;
        green *= scale;
        blue *= scale;

        red = Math.min(red, 255);
        green = Math.min(green, 255);
        blue = Math.min(blue, 255);

        float[] hsv = new float[3];
        Color.RGBToHSV((int) red, (int) green, (int) blue, hsv);

        if (red > 100 && red > green && red > blue) {
            return DetectedColor.RED;
        } else if (blue > 100 && blue > red && blue > green) {
            return DetectedColor.BLUE;
        } else if (red > 100 && green > 100 && blue < 100) {
            return DetectedColor.YELLOW;
        } else {
            return DetectedColor.UNKNOWN;
        }
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
            motor.setPower(-0.5);
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
        public static final double p = -0.025, i = 0, d = -0.00025;
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
                if (Math.abs(motor.getVelocity()) < 10) { // Assuming primary motor velocity indicates stop
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

        // Corrected getDirectionalSubtractors
        private List<double[]> getDirectionalSubtractors(
                double primaryX, double primaryY, double primaryW, double primaryH,
                List<LLResultTypes.DetectorResult> otherDetections) {

            List<double[]> subtractionRois = new ArrayList<>();
            for (LLResultTypes.DetectorResult extraDet : otherDetections) {
                // Use getTargetCorners() which returns List<List<Double>>
                List<List<Double>> extraDetCorners = extraDet.getTargetCorners();
                if (extraDetCorners == null || extraDetCorners.isEmpty()) {
                    continue;
                }

                double[] extraDetBox = getBoundingBox(extraDetCorners);
                double ex_x = extraDetBox[0];
                double ex_y = extraDetBox[1];
                double ex_w = extraDetBox[2];
                double ex_h = extraDetBox[3];

                double intersect_x1 = Math.max(primaryX, ex_x);
                double intersect_y1 = Math.max(primaryY, ex_y);
                double intersect_x2 = Math.min(primaryX + primaryW, ex_x + ex_w);
                double intersect_y2 = Math.min(primaryY + primaryH, ex_y + ex_h);

                if (intersect_x2 > intersect_x1 && intersect_y2 > intersect_y1) {
                    double intersect_w = intersect_x2 - intersect_x1;
                    double intersect_h = intersect_y2 - intersect_y1;
                    subtractionRois.add(new double[]{intersect_x1, intersect_y1, intersect_w, intersect_h});
                }
            }
            return subtractionRois;
        }

        public boolean collectSamples() {
            if (!isCollectingSamples) return false;

            limelight.pipelineSwitch(PIPE_A);
            try { Thread.sleep(20); } catch (InterruptedException e) { Thread.currentThread().interrupt(); return false;}

            LLResult resultA = limelight.getLatestResult();
            if (resultA == null || resultA.getDetectorResults() == null) {
                return false;
            }

            List<LLResultTypes.DetectorResult> detections = resultA.getDetectorResults();
            if (detections.isEmpty()) {
                return false;
            }

            LLResultTypes.DetectorResult primaryDetection = null;
            List<LLResultTypes.DetectorResult> extraDetections = new ArrayList<>();
            List<LLResultTypes.DetectorResult> tempPrimaryCandidates = new ArrayList<>();

            for (LLResultTypes.DetectorResult d : detections) {
                if (d.getClassName().equalsIgnoreCase(PRIMARY_CLASS)) {
                    tempPrimaryCandidates.add(d);
                } else {
                    extraDetections.add(d);
                }
            }

            if (tempPrimaryCandidates.isEmpty()) {
                return false;
            }

            primaryDetection = Collections.max(tempPrimaryCandidates, Comparator.comparingDouble(d -> {
                // Use getTargetCorners() here
                double[] box = getBoundingBox(d.getTargetCorners());
                return box[2] * box[3]; // w * h
            }));

            for (LLResultTypes.DetectorResult cand : tempPrimaryCandidates) {
                if (cand != primaryDetection) {
                    extraDetections.add(cand);
                }
            }

            // Use getTargetCorners() for primaryDetection as well
            double[] primaryBox = getBoundingBox(primaryDetection.getTargetCorners());
            double p_x0 = primaryBox[0];
            double p_y0 = primaryBox[1];
            double p_w0 = primaryBox[2];
            double p_h0 = primaryBox[3];

            double color_code = COLOR_MAP.getOrDefault(primaryDetection.getClassName().toLowerCase(), 1.0);

            List<double[]> subtractionRois = getDirectionalSubtractors(p_x0, p_y0, p_w0, p_h0, extraDetections);

            ArrayList<Double> llRobotDataList = new ArrayList<>();
            llRobotDataList.add(p_x0);
            llRobotDataList.add(p_y0);
            llRobotDataList.add(p_w0);
            llRobotDataList.add(p_h0);
            llRobotDataList.add(color_code);

            for (double[] strip : subtractionRois) {
                for (double val : strip) {
                    llRobotDataList.add(val);
                }
            }
            double[] llRobotData = llRobotDataList.stream().mapToDouble(Double::doubleValue).toArray();

            limelight.pipelineSwitch(PIPE_B);
            limelight.updatePythonInputs(llRobotData);
            try { Thread.sleep(30); } catch (InterruptedException e) { Thread.currentThread().interrupt(); return false;}

            LLResult resultB = limelight.getLatestResult();
            if (resultB == null || resultB.getPythonOutput() == null) {
                return false;
            }

            double[] pyOut = resultB.getPythonOutput();

            if (pyOut.length >= 3) {
                double forwardDist = pyOut[0];
                double horizontalDist = pyOut[1];
                double angleVal = pyOut[2]; // Renamed to avoid conflict

                forwardMeasurements.add(forwardDist);
                horizontalMeasurements.add(horizontalDist);
                angleMeasurements.add(angleVal);

                if (forwardMeasurements.size() > maxSamples) forwardMeasurements.remove(0);
                if (horizontalMeasurements.size() > maxSamples) horizontalMeasurements.remove(0);
                if (angleMeasurements.size() > maxSamples) angleMeasurements.remove(0);

                telemetry.addData("Limelight Output", String.format("Dist: %.1f, Hoz: %.1f, Ang: %.1f", forwardDist, horizontalDist, angleVal));
                return forwardMeasurements.size() == maxSamples;
            } else {
                return false;
            }
        }

        public Vector2d getAveragePose() {
            if (horizontalMeasurements.isEmpty() || forwardMeasurements.isEmpty()) {
                telemetry.addLine("Limelight Average Pose: No samples collected.");
                return new Vector2d(99.99, 99.99);
            }

            double avgHorizontal = horizontalMeasurements.stream().mapToDouble(Double::doubleValue).average().orElse(99.99);
            double avgForward = forwardMeasurements.stream().mapToDouble(Double::doubleValue).average().orElse(99.99);

            telemetry.addData("Limelight Avg Horizontal", String.format("%.2f", avgHorizontal));
            telemetry.addData("Limelight Avg Forward", String.format("%.2f", avgForward));
            return new Vector2d(avgHorizontal, avgForward);
        }

        public double getAverageAngle() {
            if (angleMeasurements.isEmpty()) {
                telemetry.addLine("Limelight Average Angle: No samples collected.");
                return 99.99;
            }
            double avgAngle = angleMeasurements.stream().mapToDouble(Double::doubleValue).average().orElse(99.99);
            telemetry.addData("Limelight Avg Angle", String.format("%.2f", avgAngle));
            return avgAngle;
        }
    }
}