package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "AmeyLimelightTest")
@Disabled
public class AmeyLimelightTest extends LinearOpMode {

    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    // meters → inches
    private static final double M_TO_IN = 39.3701;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint  = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        telemetry.setMsTransmissionInterval(50);

        limelight.pipelineSwitch(0);  // AprilTag / MegaTag2 pipeline
        limelight.start();

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Pinpoint heading (DEGREES) ---
            pinpoint.update();
            Pose2D ppPose = pinpoint.getPosition();
            double headingDeg = AngleUnit.normalizeDegrees(
                    ppPose.getHeading(AngleUnit.DEGREES)
            );

            // Feed IMU yaw into MegaTag2
            limelight.updateRobotOrientation(headingDeg);

            // --- Limelight result ---
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                Pose3D llPose = result.getBotpose_MT2();
                Position pos = llPose.getPosition();
                YawPitchRollAngles ori = llPose.getOrientation();

                // Convert meters → inches
                double xIn = pos.x * M_TO_IN;
                double yIn = pos.y * M_TO_IN;
                double zIn = pos.z * M_TO_IN;

                double yawDeg = ori.getYaw(AngleUnit.DEGREES);

                telemetry.addLine("=== Limelight MegaTag2 Pose (INCHES) ===");
                telemetry.addData("X (in)", xIn);
                telemetry.addData("Y (in)", yIn);
                telemetry.addData("Z (in)", zIn);
                telemetry.addData("Yaw (deg)", yawDeg);
            }
            else {
                telemetry.addLine("No valid Limelight pose");
            }

            // --- Also show Pinpoint pose (for debugging only) ---
            telemetry.addLine();
            telemetry.addLine("=== Pinpoint Odometry ===");
            telemetry.addData("PP X (mm)", ppPose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM));
            telemetry.addData("PP Y (mm)", ppPose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM));
            telemetry.addData("PP Heading (deg)", headingDeg);

            telemetry.update();
        }
    }
}
