package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Motor Overcurrent Test (Java)")
public class MotorCurrent extends LinearOpMode {

    // Define the threshold for overcurrent in Amps.
    // Adjust this value based on your motor's specifications and typical usage.
    public static double CURRENT_THRESHOLD_AMPS = 4.5; // Example value, change as needed

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the motor. Replace "motorName" with the actual name
        // configured on your robot's control hub.
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "extendo");

        // You might want to set the motor to run without encoders if you're
        // just testing current draw under load, or set it to a specific mode.
        // motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // --- Method 1: Using motor.isOverCurrent() ---
            boolean isOverCurrentDirect = motor.isOverCurrent();

            // --- Method 2: Using getCurrent() and a threshold ---
            double currentAmps = motor.getCurrent(CurrentUnit.AMPS);
            boolean isOverCurrentThreshold = currentAmps > CURRENT_THRESHOLD_AMPS;

            // --- Telemetry ---
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addLine("--- Using isOverCurrent() ---");
            telemetry.addData("motor.isOverCurrent()", isOverCurrentDirect);

            telemetry.addLine("\n--- Using getCurrentAmps() & Threshold ---");
            telemetry.addData("Current (Amps)", String.format("%.2f", currentAmps));
            telemetry.addData("Current Threshold (Amps)", String.format("%.2f", CURRENT_THRESHOLD_AMPS));
            telemetry.addData("Over Current (Threshold)", isOverCurrentThreshold);

            // Provide feedback if either method detects overcurrent
            if (isOverCurrentDirect || isOverCurrentThreshold) {
                telemetry.addLine("\nWARNING: MOTOR OVERCURRENT DETECTED!");
            } else {
                telemetry.addLine("\nMotor current within normal limits.");
            }

            telemetry.update();

            // --- Motor Control (Example) ---
            // You'll need to control the motor to actually draw current.
            // For example, you could use gamepad inputs to control the motor power.
            // This allows you to manually stall the motor or put it under load.

            double motorPower = gamepad1.left_stick_y; // Use left stick Y for power
            motor.setPower(motorPower);

            // It's good practice to have a small delay in the loop
            // to prevent spamming the control hub.
            sleep(50);
        }

        // Stop the motor when the OpMode is finished
        motor.setPower(0.0);
    }
}