package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * 2025 Teleop - Main teleop control for this year's robot
 * 
 * This is the main teleop file for the 2025 season.
 * Add your robot's specific hardware and controls here.
 */
@TeleOp(name = "2025 Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {

    // GoBILDA Mecanum Drive Motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    
    // Add other hardware as needed
    // private DcMotor liftMotor = null;
    // private Servo clawServo = null;
    // private Servo armServo = null;

    // GoBILDA Mecanum Drive constants
    private static final double DRIVE_SPEED = 0.8;
    private static final double TURN_SPEED = 0.6;
    private static final double STRAFE_SPEED = 0.7;
    private static final double PRECISION_MODE_MULTIPLIER = 0.4; // Precision mode speed reduction

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press START to begin teleop");
        telemetry.update();
        
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get gamepad inputs - using only left stick for all movement
            double leftStickY = -gamepad1.left_stick_y;  // Forward/backward
            double leftStickX = gamepad1.left_stick_x;  // Strafe left/right
            // No right stick - all movement from left stick only

            // Check for precision mode (right bumper)
            boolean precisionMode = gamepad1.right_bumper;
            double speedMultiplier = precisionMode ? PRECISION_MODE_MULTIPLIER : 1.0;

            // GoBILDA Mecanum wheel calculations using left stick only
            // Forward/backward: leftStickY
            // Strafe left/right: leftStickX  
            // No turning - pure mecanum movement only
            double leftFrontPower = leftStickY + leftStickX;
            double rightFrontPower = leftStickY - leftStickX;
            double leftBackPower = leftStickY - leftStickX;
            double rightBackPower = leftStickY + leftStickX;

            // Normalize the wheel speeds to prevent motor saturation
            double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Apply speed multiplier and power to GoBILDA motors
            leftFrontDrive.setPower(leftFrontPower * DRIVE_SPEED * speedMultiplier);
            rightFrontDrive.setPower(rightFrontPower * DRIVE_SPEED * speedMultiplier);
            leftBackDrive.setPower(leftBackPower * DRIVE_SPEED * speedMultiplier);
            rightBackDrive.setPower(rightBackPower * DRIVE_SPEED * speedMultiplier);

            // Add other subsystem controls here
            // controlLift();
            // controlClaw();
            // controlArm();

            // Display telemetry
            updateTelemetry();

            // Pause for metronome tick (40ms)
            sleep(40);
        }
    }

    /**
     * Initialize GoBILDA mecanum drive hardware
     */
    private void initializeHardware() {
        // Initialize GoBILDA drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // GoBILDA motor directions for mecanum wheels
        // Adjust these if your robot moves in unexpected directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set GoBILDA motor modes for teleop
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior for GoBILDA motors
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize other hardware as needed
        // liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        // clawServo = hardwareMap.get(Servo.class, "clawServo");
        // armServo = hardwareMap.get(Servo.class, "armServo");
    }

    /**
     * Control lift mechanism (example)
     */
    private void controlLift() {
        // Example lift control - uncomment and modify as needed
        /*
        if (gamepad2.dpad_up) {
            liftMotor.setPower(0.5);
        } else if (gamepad2.dpad_down) {
            liftMotor.setPower(-0.5);
        } else {
            liftMotor.setPower(0);
        }
        */
    }

    /**
     * Control claw mechanism (example)
     */
    private void controlClaw() {
        // Example claw control - uncomment and modify as needed
        /*
        if (gamepad2.a) {
            clawServo.setPosition(0.0); // Close
        } else if (gamepad2.b) {
            clawServo.setPosition(1.0); // Open
        }
        */
    }

    /**
     * Control arm mechanism (example)
     */
    private void controlArm() {
        // Example arm control - uncomment and modify as needed
        /*
        double armPosition = armServo.getPosition();
        if (gamepad2.left_bumper) {
            armPosition += 0.01;
        } else if (gamepad2.right_bumper) {
            armPosition -= 0.01;
        }
        armPosition = Range.clip(armPosition, 0.0, 1.0);
        armServo.setPosition(armPosition);
        */
    }

    /**
     * Update telemetry display for GoBILDA mecanum drive
     */
    private void updateTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive Mode", "GoBILDA Mecanum");
        telemetry.addData("Precision Mode", gamepad1.right_bumper ? "ON" : "OFF");
        
        // GoBILDA motor power telemetry
        telemetry.addData("LF Power", String.format("%.2f", leftFrontDrive.getPower()));
        telemetry.addData("RF Power", String.format("%.2f", rightFrontDrive.getPower()));
        telemetry.addData("LB Power", String.format("%.2f", leftBackDrive.getPower()));
        telemetry.addData("RB Power", String.format("%.2f", rightBackDrive.getPower()));
        
        // Add other subsystem telemetry as needed
        // telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
        // telemetry.addData("Claw Position", String.format("%.2f", clawServo.getPosition()));
        
        telemetry.update();
    }
}
