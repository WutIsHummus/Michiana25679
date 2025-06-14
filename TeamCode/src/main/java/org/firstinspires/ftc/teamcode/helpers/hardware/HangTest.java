package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Hang Mechanism Test with Current")
public class HangTest extends LinearOpMode {

    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx liftMotorLeft;
    private DcMotorEx liftMotorRight;
    private Servo ptor, ptol;

    private final double PTOL_UNLOCKED_POSITION = 0.44;
    private final double PTOL_LOCKED_POSITION = 0.55;
    private final double PTOR_UNLOCKED_POSITION = 0.6;
    private final double PTOR_LOCKED_POSITION = 0.45;

    private final double HANG_MOTOR_SPEED_DOWN = -1.0;
    private final double LIFT_ASSIST_POWER_UP = 0.48;
    private final double LIFT_ASSIST_POWER_DOWN = -0.48;

    private boolean ptoLocked = false;
    private boolean gamepad1XPreviouslyPressed = false;
    private double currentHangMotorPower = 0.0;
    private double currentLiftAssistPower = 0.0;

    public static double CURRENT_THRESHOLD_AMPS = 5.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        try {
            motorFrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
            motorFrontRight = hardwareMap.get(DcMotorEx.class, "FR");
            motorBackLeft = hardwareMap.get(DcMotorEx.class, "BL");
            motorBackRight = hardwareMap.get(DcMotorEx.class, "BR");

            liftMotorLeft = hardwareMap.get(DcMotorEx.class, "liftl");
            liftMotorRight = hardwareMap.get(DcMotorEx.class, "liftr");

            ptol = hardwareMap.get(Servo.class, "ptol");
            ptor = hardwareMap.get(Servo.class, "ptor");

            motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

            liftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            ptol.setPosition(PTOL_UNLOCKED_POSITION);
            ptor.setPosition(PTOR_UNLOCKED_POSITION);
            ptoLocked = false;

            telemetry.addData("Status", "Initialized");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Status", "Error Initializing Hardware");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            while (!isStopRequested() && opModeIsActive()) {
                idle();
            }
            return;
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            boolean gamepad1XCurrentlyPressed = gamepad1.x;
            if (gamepad1XCurrentlyPressed && !gamepad1XPreviouslyPressed) {
                ptoLocked = !ptoLocked;
                if (ptoLocked) {
                    ptor.setPosition(PTOR_LOCKED_POSITION);
                    ptol.setPosition(PTOL_LOCKED_POSITION);
                } else {
                    ptor.setPosition(PTOR_UNLOCKED_POSITION);
                    ptol.setPosition(PTOL_UNLOCKED_POSITION);
                }
            }
            gamepad1XPreviouslyPressed = gamepad1XCurrentlyPressed;

            currentHangMotorPower = 0.0;
            currentLiftAssistPower = 0.0;

            if (gamepad1.dpad_up) {
                currentLiftAssistPower = LIFT_ASSIST_POWER_UP;
            } else if (gamepad1.dpad_down) {
                currentHangMotorPower = HANG_MOTOR_SPEED_DOWN;
                currentLiftAssistPower = LIFT_ASSIST_POWER_DOWN;
            }

            motorFrontLeft.setPower(currentHangMotorPower);
            motorFrontRight.setPower(currentHangMotorPower);
            motorBackLeft.setPower(currentHangMotorPower);
            motorBackRight.setPower(currentHangMotorPower);

            liftMotorLeft.setPower(currentLiftAssistPower);
            liftMotorRight.setPower(currentLiftAssistPower);

            telemetry.addData("PTO State", ptoLocked ? "LOCKED" : "UNLOCKED");
            telemetry.addData("Hang Power", currentHangMotorPower);
            telemetry.addData("Lift Power", currentLiftAssistPower);
            telemetry.addLine();

            addMotorCurrentTelemetry("FL", motorFrontLeft);
            addMotorCurrentTelemetry("FR", motorFrontRight);
            addMotorCurrentTelemetry("BL", motorBackLeft);
            addMotorCurrentTelemetry("BR", motorBackRight);
            addMotorCurrentTelemetry("LiftL", liftMotorLeft);
            addMotorCurrentTelemetry("LiftR", liftMotorRight);

            telemetry.addLine();
            telemetry.addData("Current Threshold", String.format("%.2f A", CURRENT_THRESHOLD_AMPS));
            telemetry.update();

            sleep(20);
        }
    }

    private void addMotorCurrentTelemetry(String label, DcMotorEx motor) {
        double current = motor.getCurrent(CurrentUnit.AMPS);
        boolean isOverCurrent = current > CURRENT_THRESHOLD_AMPS;

        telemetry.addLine(String.format("Motor %s:", label));
        telemetry.addData("  Current", String.format("%.2f A", current));
        telemetry.addData("  Over Threshold?", isOverCurrent);
        if (isOverCurrent) {
            telemetry.addLine("  !! OVERCURRENT WARNING !!");
        }
    }
}
