package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
// TODO: PoseUpdater removed in PedroPathing 2.0
// import com.pedropathing.telemetry.PoseUpdater;
// import com.pedropathing.telemetry.DashboardPoseTracker;
// import com.pedropathing.telemetry.Drawing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import com.pedropathing.follower.Follower;

@Config
@TeleOp(name = "Limelight Relocalization Test")
public class LimelightRelocalizationTest extends OpMode {
    
    private Follower follower;  // Follower includes Pinpoint localization (PedroPathing 2.0)
    private Telemetry telemetryA;
    
    private DcMotorEx fl, fr, bl, br;
    private Limelight3A limelight;
    
    // Relocalization settings
    public static boolean ENABLE_AUTO_RELOCALIZATION = false;  // Default OFF for testing
    public static double RELOCALIZE_INTERVAL_SECONDS = 2.0;
    
    private ElapsedTime relocalizeTimer;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    
    // Statistics
    private int relocalizeAttempts = 0;
    private int relocalizeSuccesses = 0;
    private Pose lastOdometryPose = null;
    private Pose lastLimelightPose = null;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(25);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize Follower with Pinpoint localizer (PedroPathing 2.0)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
        
        // Initialize drive motors
        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);  // Use pipeline 0 (AprilTag)
            limelight.start();
            telemetryA.addLine("✓ Limelight initialized");
        } catch (Exception e) {
            telemetryA.addLine("❌ Limelight not found: " + e.getMessage());
            limelight = null;
        }
        
        relocalizeTimer = new ElapsedTime();

        telemetryA.addLine("=================================");
        telemetryA.addLine("LIMELIGHT RELOCALIZATION TEST");
        telemetryA.addLine("=================================");
        telemetryA.addLine("");
        telemetryA.addLine("Controls:");
        telemetryA.addLine("  D-Pad Up: Manual relocalize NOW");
        telemetryA.addLine("  D-Pad Down: Toggle auto-relocalize");
        telemetryA.addLine("  Right Stick: Drive");
        telemetryA.addLine("  Left Stick X: Rotate");
        telemetryA.addLine("");
        telemetryA.addLine("Requirements:");
        telemetryA.addLine("  - Limelight AprilTag pipeline active");
        telemetryA.addLine("  - Field map uploaded to Limelight");
        telemetryA.addLine("  - Limelight pose offset configured");
        telemetryA.addLine("  - At least 1 AprilTag visible");
        telemetryA.update();
    }

    @Override
    public void loop() {
        // Update odometry
        // Update Follower (which updates Pinpoint localization) - PedroPathing 2.0
        follower.update();
        
        // Get current position from Follower (Pinpoint localization)
        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();
        
        // Update dashboard
        // TODO: Drawing removed
        // Drawing.drawRobot(currentPose, "#4CAF50");
        // TODO: Drawing removed
        // Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50FF");
        // TODO: Drawing removed
        // Drawing.sendPacket();

        // ==================== DRIVE CONTROLS ====================
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x * 1.1;
        double rx = gamepad1.left_stick_x;
        
        double scale = 1.0;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        
        double flPower = (y + x + rx) / denominator * scale;
        double blPower = (y - x + rx) / denominator * scale;
        double frPower = (y - x - rx) / denominator * scale;
        double brPower = (y + x - rx) / denominator * scale;
        
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        // ==================== RELOCALIZATION CONTROL ====================
        boolean relocalized = false;
        String relocalizeResult = "";
        
        // Toggle auto-relocalization with D-Pad Down
        boolean currentDpadDown = gamepad1.dpad_down;
        if (currentDpadDown && !lastDpadDown) {
            ENABLE_AUTO_RELOCALIZATION = !ENABLE_AUTO_RELOCALIZATION;
        }
        lastDpadDown = currentDpadDown;
        
        // Manual relocalization with D-Pad Up
        boolean currentDpadUp = gamepad1.dpad_up;
        if (currentDpadUp && !lastDpadUp) {
            relocalized = attemptRelocalization();
            relocalizeResult = relocalized ? "✓ SUCCESS (Manual)" : "❌ FAILED (Manual)";
        }
        lastDpadUp = currentDpadUp;
        
        // Automatic periodic relocalization
        if (ENABLE_AUTO_RELOCALIZATION && relocalizeTimer.seconds() > RELOCALIZE_INTERVAL_SECONDS) {
            if (attemptRelocalization()) {
                relocalized = true;
                relocalizeResult = "✓ AUTO";
            }
            relocalizeTimer.reset();
        }

        // ==================== TELEMETRY ====================
        telemetryA.addLine("=== RELOCALIZATION TEST ===");
        telemetryA.addData("Auto Relocalize", ENABLE_AUTO_RELOCALIZATION ? "ENABLED" : "DISABLED");
        if (ENABLE_AUTO_RELOCALIZATION) {
            telemetryA.addData("Next Auto in", "%.1f seconds", RELOCALIZE_INTERVAL_SECONDS - relocalizeTimer.seconds());
        }
        telemetryA.addData("Attempts", relocalizeAttempts);
        telemetryA.addData("Successes", relocalizeSuccesses);
        telemetryA.addData("Success Rate", relocalizeAttempts > 0 ? 
            String.format("%.0f%%", (100.0 * relocalizeSuccesses / relocalizeAttempts)) : "N/A");
        
        if (relocalized) {
            telemetryA.addData("", "");
            telemetryA.addData("🎯 RELOCALIZED!", relocalizeResult);
        }
        
        telemetryA.addData("", "");
        telemetryA.addLine("=== CURRENT ODOMETRY POSE ===");
        telemetryA.addData("X", "%.2f inches", currentX);
        telemetryA.addData("Y", "%.2f inches", currentY);
        telemetryA.addData("Heading", "%.1f° (%.3f rad)", Math.toDegrees(currentHeading), currentHeading);
        
        if (lastLimelightPose != null) {
            telemetryA.addData("", "");
            telemetryA.addLine("=== LAST LIMELIGHT POSE (Pinpoint Format) ===");
            telemetryA.addData("X", "%.2f inches (Pinpoint)", lastLimelightPose.getX());
            telemetryA.addData("Y", "%.2f inches (Pinpoint)", lastLimelightPose.getY());
            telemetryA.addData("Heading", "%.1f°", Math.toDegrees(lastLimelightPose.getHeading()));
            telemetryA.addLine("  (Already converted from Limelight center-origin)");
            
            if (lastOdometryPose != null) {
                double deltaX = currentX - lastOdometryPose.getX();
                double deltaY = currentY - lastOdometryPose.getY();
                double deltaHeading = Math.toDegrees(currentHeading - lastOdometryPose.getHeading());
                
                telemetryA.addData("", "");
                telemetryA.addLine("=== CORRECTION APPLIED ===");
                telemetryA.addData("ΔX", "%.2f inches", deltaX);
                telemetryA.addData("ΔY", "%.2f inches", deltaY);
                telemetryA.addData("ΔHeading", "%.1f°", deltaHeading);
            }
        }
        
        // Check Limelight status
        telemetryA.addData("", "");
        telemetryA.addLine("=== LIMELIGHT STATUS ===");
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                telemetryA.addData("Valid", result.isValid() ? "✓ YES" : "❌ NO");
                
                Pose3D botpose = result.getBotpose();
                if (botpose != null && botpose.getPosition() != null) {
                    double rawX = botpose.getPosition().x * 39.3701;  // Limelight coords (center origin)
                    double rawY = botpose.getPosition().y * 39.3701;
                    double pinpointX = rawX + 72.0;  // Converted to Pinpoint (corner origin)
                    double pinpointY = rawY + 72.0;
                    
                    telemetryA.addLine("  Raw Limelight (center=0):");
                    telemetryA.addData("    X", "%.2f inches", rawX);
                    telemetryA.addData("    Y", "%.2f inches", rawY);
                    telemetryA.addLine("  Converted to Pinpoint (corner=0):");
                    telemetryA.addData("    X", "%.2f inches (+72)", pinpointX);
                    telemetryA.addData("    Y", "%.2f inches (+72)", pinpointY);
                    telemetryA.addData("  Yaw", "%.1f°", botpose.getOrientation().getYaw());
                } else {
                    telemetryA.addData("Botpose", "❌ Not available");
                }
            } else {
                telemetryA.addData("Result", "❌ NULL");
            }
        } else {
            telemetryA.addData("Limelight", "❌ NOT INITIALIZED");
        }
        
        telemetryA.addData("", "");
        telemetryA.addLine("=== CONTROLS ===");
        telemetryA.addData("D-Pad Up", "Relocalize now");
        telemetryA.addData("D-Pad Down", "Toggle auto (" + (ENABLE_AUTO_RELOCALIZATION ? "ON" : "OFF") + ")");
        
        telemetryA.update();
    }
    
    /**
     * Attempt to relocalize using Limelight's AprilTag detection (MegaTag/botpose)
     * @return true if relocalization successful, false otherwise
     */
    private boolean attemptRelocalization() {
        relocalizeAttempts++;
        
        if (limelight == null) {
            return false;
        }
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return false;
        }
        
        // Get botpose from Limelight (field-relative robot position)
        Pose3D botpose = result.getBotpose();
        if (botpose == null || botpose.getPosition() == null) {
            return false;
        }
        
        // Extract position from botpose (in meters, convert to inches)
        double limelightRawX = botpose.getPosition().x * 39.3701;  // meters to inches (Limelight coords)
        double limelightRawY = botpose.getPosition().y * 39.3701;  // meters to inches (Limelight coords)
        
        // ═══════════════════════════════════════════════════════════
        // COORDINATE CONVERSION: Limelight → Pinpoint
        // ═══════════════════════════════════════════════════════════
        // Limelight uses field CENTER as origin (0,0)
        // Pinpoint uses BOTTOM-LEFT corner as origin (0,0)
        // Field is 144" × 144", so center is at (72, 72) in Pinpoint coords
        // Conversion: pinpoint = limelight + 72
        // ═══════════════════════════════════════════════════════════
        double limelightX = limelightRawX + 72.0;  // CONVERTED to Pinpoint coords
        double limelightY = limelightRawY + 72.0;  // CONVERTED to Pinpoint coords
        double limelightHeading = Math.toRadians(botpose.getOrientation().getYaw());  // degrees to radians
        
        // Store poses for comparison
        // TODO: PoseUpdater removed
        // TODO: PoseUpdater removed
        // // lastOdometryPose = poseUpdater.getPose();
        
        // Update PoseUpdater with Limelight's pose estimate
        try {
            Pose limelightPose = new Pose(
                limelightX,
                limelightY,
                limelightHeading
            );
            
            lastLimelightPose = limelightPose;
            
            // Update the pose using Follower (PedroPathing 2.0)
            follower.setPose(limelightPose);
            
            relocalizeSuccesses++;
            return true;
        } catch (Exception e) {
            // If setPose doesn't work, try alternative methods
            telemetryA.addData("Relocalize Error", e.getMessage());
            return false;
        }
    }

    @Override
    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        
        if (limelight != null) {
            limelight.stop();
        }
    }
}

