package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.util.LimelightDashboardStream;

import java.util.List;

@TeleOp(name = "Test: Limelight3A AprilTags", group = "Test")
public class AprilTagLimelightTest extends LinearOpMode {

    private Limelight3A limelight;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // Switch to pipeline number 0

        // Dashboard setup and stream
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // Replace with your Limelight IP or hostname
        String limelightHost = "limelight.local"; // or e.g. "192.168.1.11"
        dashboard.startCameraStream((CameraStreamSource) new LimelightDashboardStream(limelightHost), 24);

        telemetry.setMsTransmissionInterval(11);
        limelight.start();

        // Optional: faster poll rate for testing; safe default if unsupported by firmware
        try { limelight.setPollRateHz(250); } catch (Throwable ignored) {}

        telemetry.addLine("> Limelight ready. Ensure an AprilTag (fiducial) pipeline is active in the Limelight UI.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL Name", status.getName());
            telemetry.addData("LL Stats", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("Latency(ms)", captureLatency + targetingLatency);
                telemetry.addData("Parse(ms)", parseLatency);
                telemetry.addData("Valid", result.isValid());
                telemetry.addData("Botpose", botpose);

                // Focus on AprilTags (fiducials)
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                telemetry.addData("Fiducials", fiducials.size());
                int idx = 0;
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    telemetry.addData(String.format("Tag[%d]", idx++),
                            String.format("ID:%d Family:%s Xdeg:%.1f Ydeg:%.1f Area:%.2f", 
                                    fr.getFiducialId(), fr.getFamily(),
                                    fr.getTargetXDegrees(), fr.getTargetYDegrees(),
                                    fr.getTargetArea()));
                }
            } else {
                telemetry.addLine("No Limelight result yet.");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}


