package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * LimelightRobot - Similar to BarrelRobot but uses Limelight for relocalization
 * Handles PedroPathing follower and automatic AprilTag-based relocalization
 */
@Config
public class LimelightRobot {
    
    private Limelight3A limelight;
    public Follower follower;  // Follower includes Pinpoint localization (PedroPathing 2.0)
    
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    
    // Coordinate conversion - Field dimensions
    private static final double FIELD_SIZE_INCHES = 144.0;
    private static final double CENTER_OFFSET = FIELD_SIZE_INCHES / 2.0;  // 72 inches
    
    public LimelightRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        
        // Initialize Follower with Pinpoint localizer (PedroPathing 2.0)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
        
        this.initLimelight();
    }
    
    /**
     * Call this every loop - updates odometry (NO auto-relocalization)
     * PedroPathing 2.0 - uses Follower for Pinpoint localization
     */
    public void onTick() {
        // Update Follower (which updates Pinpoint localization) - PedroPathing 2.0
        follower.update();
    }
    
    /**
     * Get Limelight pose without relocalization - for comparison
     */
    public Pose getLimelightPose() {
        if (limelight == null) {
            return null;
        }
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        
        Pose3D botpose = result.getBotpose();
        if (botpose == null || botpose.getPosition() == null) {
            return null;
        }
        
        double limelightRawX_meters = botpose.getPosition().x;
        double limelightRawY_meters = botpose.getPosition().y;
        
        if (Math.abs(limelightRawX_meters) < 0.001 && Math.abs(limelightRawY_meters) < 0.001) {
            return null;
        }
        
        double limelightRawX = limelightRawX_meters * 39.3701;
        double limelightRawY = limelightRawY_meters * 39.3701;
        double pinpointX = limelightRawX + CENTER_OFFSET;
        double pinpointY = limelightRawY + CENTER_OFFSET;
        double pinpointHeading = Math.toRadians(botpose.getOrientation().getYaw());
        pinpointHeading = (pinpointHeading + 2 * Math.PI) % (2 * Math.PI);
        
        return new Pose(pinpointX, pinpointY, pinpointHeading);
    }
    
    /**
     * Initialize Limelight
     */
    private void initLimelight() {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);  // Use pipeline 0 (AprilTag)
            limelight.start();
            telemetry.addLine("✓ Limelight initialized");
        } catch (Exception e) {
            telemetry.addLine("❌ Limelight not found: " + e.getMessage());
            limelight = null;
        }
    }
    
    /**
     * Relocalize from AprilTag - similar to BarrelRobot but uses Limelight
     */
    public void relocaliseFromAprilTag() {
        if (limelight == null) {
            telemetry.addData("AprilTag", "Not found");
            return;
        }
        
        LLResult result = limelight.getLatestResult();
        
        if (result == null || !result.isValid()) {
            telemetry.addData("AprilTag", "Not found");
            return;
        }
        
        // Get botpose from Limelight (MegaTag localization)
        Pose3D botpose = result.getBotpose();
        if (botpose == null || botpose.getPosition() == null) {
            telemetry.addData("AprilTag", "Not found");
            return;
        }
        
        // Extract raw Limelight coordinates (meters, center-origin)
        double limelightRawX_meters = botpose.getPosition().x;
        double limelightRawY_meters = botpose.getPosition().y;
        
        // Check if data is valid (non-zero)
        if (Math.abs(limelightRawX_meters) < 0.001 && Math.abs(limelightRawY_meters) < 0.001) {
            telemetry.addData("AprilTag", "Not found");
            return;
        }
        
        telemetry.addData("AprilTag", "Found");
        
        // Convert to inches
        double limelightRawX = limelightRawX_meters * 39.3701;
        double limelightRawY = limelightRawY_meters * 39.3701;
        
        // Convert from Limelight coordinates (center-origin) to Pinpoint (corner-origin)
        // Limelight: (0,0) = field center
        // Pinpoint: (0,0) = bottom-left corner
        // Conversion: pinpoint = limelight + 72
        double pinpointX = limelightRawX + CENTER_OFFSET;
        double pinpointY = limelightRawY + CENTER_OFFSET;
        double pinpointHeading = Math.toRadians(botpose.getOrientation().getYaw());
        
        // Normalize heading to 0 to 2π (same as BarrelRobot)
        pinpointHeading = (pinpointHeading + 2 * Math.PI) % (2 * Math.PI);
        
        // Set pose using Follower (PedroPathing 2.0)
        follower.setPose(new Pose(pinpointX, pinpointY, pinpointHeading));
    }
    
    /**
     * This function adds a bezier line of the robot's historical positions. Call this each frame
     */
    // TODO: Drawing and PoseUpdater removed in PedroPathing 2.0
    private void drawCurrentAndHistory() {
        // Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        // drawCurrent();
    }
    
    /**
     * This function updates the Robot's position on the field to what the PoseUpdater thinks it's position is.
     */
    private void drawCurrent() {
        // TODO: Drawing and PoseUpdater removed in PedroPathing 2.0
        // try {
        //     Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        //     Drawing.sendPacket();
        // } catch (Exception e) {
        //     throw new RuntimeException("Drawing failed " + e);
        // }
    }
}

