package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Calibration Tool", group = "Test")
public class LimelightCalibration extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("CALIBRATION MODE");
        telemetry.addLine("Place robot at known distances (e.g., 24, 36, 48, 60 inches)");
        telemetry.addLine("Record the 'ty' value for each distance.");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();

            telemetry.addData("Raw ty", "%.4f", ty);
            telemetry.addData("Inverted ty", "%.4f", -ty);

            // Help user verify the trend
            telemetry.addLine("\n--- TREND CHECK ---");
            telemetry.addLine("As you move AWAY, 'Raw ty' should get CLOSER to 0.");
            telemetry.addLine("If it gets MORE NEGATIVE, something is wrong!");
        } else {
            telemetry.addData("Status", "No Tag Detected");
        }

        telemetry.update();
    }
}
