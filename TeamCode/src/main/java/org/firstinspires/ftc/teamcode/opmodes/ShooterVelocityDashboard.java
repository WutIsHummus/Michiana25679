package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ShooterVelocityDashboard", group = "Test")
public class ShooterVelocityDashboard extends LinearOpMode {

    // GoBILDA bare motor: 28 ticks per revolution
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        DcMotorEx shootl = hardwareMap.get(DcMotorEx.class, "shootl");

        shootr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Combine RC telemetry + Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        shootr.setPower(0.67);
        shootl.setPower(-0.670);

        while (opModeIsActive()) {
            // Get ticks/sec from motor encoder
            double ticksPerSec = shootr.getVelocity();

            // Convert to rotations per second
            double rotationsPerSec = ticksPerSec / TICKS_PER_REV;

            telemetry.addData("Shootr Velocity (RPS)", rotationsPerSec);
            telemetry.update();
        }

        shootr.setPower(0);
        shootl.setPower(0);
    }
}



