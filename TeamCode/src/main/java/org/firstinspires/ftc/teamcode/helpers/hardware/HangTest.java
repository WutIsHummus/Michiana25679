package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
// Assuming you might have separate lift motors
// import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Hang Mechanism Test")
public class HangTest extends LinearOpMode {

    // --- Hardware Declarations ---
    // Hang Motors (assuming two, adjust as needed)
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

    private final double HANG_MOTOR_SPEED_DOWN = -1.0; // Max speed for hang down (reverse)


    // Lift Motor Assist Power
    private final double LIFT_ASSIST_POWER_UP = 0.48;   // Power for lift motors when hang goes UP
    private final double LIFT_ASSIST_POWER_DOWN = -0.48;  // Power for lift motors when hang goes DOWN

    // --- State Variables ---
    private boolean ptoLocked = false; // Start with PTO locked
    private boolean gamepad1XPreviouslyPressed = false;
    private double currentHangMotorPower = 0.0;
    private double currentLiftAssistPower = 0.0;

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



            motorFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            motorFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
            motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
            motorBackRight.setDirection(DcMotorEx.Direction.FORWARD);

            liftMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
            liftMotorRight.setDirection(DcMotorEx.Direction.REVERSE);


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
            telemetry.addLine("Controls:");
            telemetry.addLine(" G1.X: Toggle PTO Lock/Unlock");
            telemetry.addLine(" G1.Dpad Up: Hang Up (+ Lift Assist Up)");
            telemetry.addLine(" G1.Dpad Down: Hang Down (+ Lift Assist Down)");
            telemetry.addLine("Release Dpad: Stop Hang & Lift Assist");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Status", "Error Initializing Hardware");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            // Prevent the OpMode from running if hardware fails to init
            while (!isStopRequested() && opModeIsActive()) {
                idle();
            }
            return;
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // --- PTO Lock/Unlock Control (Gamepad 1 X button) ---`
            boolean gamepad1XCurrentlyPressed = gamepad1.x;
            if (gamepad1XCurrentlyPressed && !gamepad1XPreviouslyPressed) {
                ptoLocked = !ptoLocked; // Toggle the state
                if (ptoLocked) {
                    ptor.setPosition(PTOR_LOCKED_POSITION);
                    ptol.setPosition(PTOL_LOCKED_POSITION);
                } else {
                    ptor.setPosition(PTOR_UNLOCKED_POSITION);
                    ptol.setPosition(PTOL_UNLOCKED_POSITION);
                }
            }
            gamepad1XPreviouslyPressed = gamepad1XCurrentlyPressed;

            // --- Hang and Lift Assist Control (Gamepad 1 Dpad Up/Down) ---
            currentHangMotorPower = 0.0;
            currentLiftAssistPower = 0.0;

            if (gamepad1.dpad_up) {
                // Hang UP
                currentLiftAssistPower = LIFT_ASSIST_POWER_UP;
            } else if (gamepad1.dpad_down) {
                currentHangMotorPower = HANG_MOTOR_SPEED_DOWN;
                currentLiftAssistPower = LIFT_ASSIST_POWER_DOWN;
            }

            // Apply power to motors
            motorFrontLeft.setPower(currentHangMotorPower);
            motorFrontRight.setPower(currentHangMotorPower);
            motorBackLeft.setPower(currentHangMotorPower);
            motorBackRight.setPower(currentHangMotorPower);

            liftMotorLeft.setPower(currentLiftAssistPower);
            liftMotorRight.setPower(currentLiftAssistPower);


            // --- Telemetry Update ---
            telemetry.addData("PTO State", ptoLocked ? "LOCKED" : "UNLOCKED");
            telemetry.addData("PTO Right Servo Position", String.format("%.2f", ptor.getPosition()));
            telemetry.addData("PTO Left Servo Position", String.format("%.2f", ptol.getPosition()));
            telemetry.addLine();
            telemetry.addData("Hang Dpad Up", gamepad1.dpad_up);
            telemetry.addData("Hang Dpad Down", gamepad1.dpad_down);
            telemetry.addData("Hang Motor Power", String.format("%.2f", currentHangMotorPower));
            telemetry.addLine();
            telemetry.addData("Lift Assist Power", String.format("%.2f", currentLiftAssistPower));
            telemetry.update();

            // Small delay to prevent loop from running too fast
            sleep(20);
        }
    }
}