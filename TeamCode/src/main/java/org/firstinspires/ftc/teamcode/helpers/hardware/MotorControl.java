package org.firstinspires.ftc.teamcode.helpers.hardware;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.data.Enums.DetectedColor;

import java.util.ArrayList;
import java.util.List;

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
        public static final double GRAVITY_FEEDFORWARD = 0;

        public Lift(HardwareMap hardwareMap) {
            liftController = new PIDController(p, i, d);
            motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftr"), 0.005);
            motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftl"), 0.005);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void reset() {
            motor.setPower(0);
            motor2.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            resetting = false;
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.6);
            motor2.setPower(0.6);
            resetting = true;
        }

        public void update() {
            if (resetting) {
                if (Math.abs(motor.getVelocity()) < 10) {
                    reset();
                    resetting = false;
                }
            } else {
                liftController.setPID(p, i, d);
                double pid = liftController.calculate(motor.getCurrentPosition(), targetPosition);
                double motorPower = pid;
                motor.setPower(motorPower);
                motor2.setPower(motorPower);
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
        private final int maxSamples = 1;
        private final List<Double> horizontal = new ArrayList<>();
        private final List<Double> foward = new ArrayList<>();
        private boolean isCollectingSamples;
        private final Telemetry telemetry;

        public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            telemetry.addData("limelight", "Initialized");
            limelight.start();
        }

        public void stop() {
            limelight.stop();
        }

        public void resetSamples() {
            horizontal.clear();
            foward.clear();
            isCollectingSamples = false;
        }

        public boolean isCollectingSamples() {
            return isCollectingSamples;
        }

        public void startCollectingSamples() {
            resetSamples();
            isCollectingSamples = true;
        }

        public boolean collectSamples() {
            telemetry.addData("Collecting Samples", isCollectingSamples);
            LLResult result = limelight.getLatestResult();
            telemetry.addData("Result Exists", result != null);
            telemetry.addData("Python Output Exists", result != null && result.getPythonOutput() != null);

            if (!isCollectingSamples) return false;

            if (result != null && result.getPythonOutput() != null && result.getPythonOutput().length >= 4) {
                double forwardDistance = result.getPythonOutput()[0];
                double horizontalOffset = result.getPythonOutput()[1];
                double check = result.getPythonOutput()[3];

                if (check == 1) {
                    horizontal.add(horizontalOffset);
                    foward.add(forwardDistance);
                    if (horizontal.size() > maxSamples) horizontal.remove(0);
                    if (foward.size() > maxSamples) foward.remove(0);
                    boolean enoughSamples = horizontal.size() == maxSamples && foward.size() == maxSamples;
                    telemetry.addData("Enough Samples", enoughSamples);
                    return enoughSamples;
                }
            }
            return false;
        }

        public Vector2d getAverage() {
            if (horizontal.isEmpty() || foward.isEmpty()) {
                telemetry.addData("Average Pose", "No samples collected.");
                telemetry.update();
                return new Vector2d(99.99, 99.99);
            }

            double hor = horizontal.stream().mapToDouble(Double::doubleValue).average().orElse(0);
            double ver = foward.stream().mapToDouble(Double::doubleValue).average().orElse(0);
            telemetry.addData("Average X", hor);
            telemetry.addData("Average Y", ver);
            telemetry.update();
            return new Vector2d(hor, ver);
        }
    }
}
