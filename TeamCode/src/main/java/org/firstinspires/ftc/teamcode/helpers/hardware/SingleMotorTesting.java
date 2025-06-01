package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Single Motor Test", group = "TeleOp")
public class SingleMotorTesting extends OpMode {

    // Define one servo
    DcMotorEx testServo;

    // Dashboard-tunable target position
    public static double motorPowerTarget = 0.1;
    public static String name = "1";

    @Override
    public void init() {
        // Initialize the servo with the name you gave it in the configuration
        testServo = hardwareMap.get(DcMotorEx.class, name); // Change "testServo" to match your config name

        // Combine telemetry with dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Set the servo position to the dashboard value
        testServo.setPower(motorPowerTarget);

        // Output current position to telemetry
        telemetry.addData("motor power", motorPowerTarget);
        telemetry.update();
    }
}
