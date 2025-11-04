package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
@TeleOp(name = "AprilTag Distance Test")
public class AprilTagDistanceTest extends OpMode {
    
    private Limelight3A limelight;
    
    // Configurable constants (adjustable from FTC Dashboard)
    public static double APRILTAG_HEIGHT_INCHES = 30.0;      // Height of AprilTag center
    public static double LIMELIGHT_HEIGHT_INCHES = 13.5;     // Height of Limelight lens
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 19.0;   // Upward tilt angle
    public static int TARGET_APRILTAG_ID = -1;               // -1 for any tag, or specific ID
    
    private double heightDifference;
    
    @Override
    public void init() {
        // Initialize telemetry with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.setPollRateHz(100);
        
        heightDifference = APRILTAG_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES;
        
        telemetry.addLine("AprilTag Distance Test Initialized");
        telemetry.addLine("This OpMode calculates distance to AprilTags using:");
        telemetry.addLine("  - Limelight angles (tx, ty)");
        telemetry.addLine("  - Camera height and mount angle");
        telemetry.addLine("  - Trigonometry");
        telemetry.addData("", "");
        telemetry.addLine("Adjust values on FTC Dashboard:");
        telemetry.addData("  APRILTAG_HEIGHT_INCHES", "%.1f", APRILTAG_HEIGHT_INCHES);
        telemetry.addData("  LIMELIGHT_HEIGHT_INCHES", "%.1f", LIMELIGHT_HEIGHT_INCHES);
        telemetry.addData("  LIMELIGHT_MOUNT_ANGLE_DEG", "%.1f", LIMELIGHT_MOUNT_ANGLE_DEG);
        telemetry.addData("  TARGET_APRILTAG_ID", "%d (-1 = any)", TARGET_APRILTAG_ID);
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Update height difference in case values changed on dashboard
        heightDifference = APRILTAG_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES;
        
        // Get latest Limelight result
        LLResult result = limelight.getLatestResult();
        
        if (result == null) {
            telemetry.addData("Status", "⚠️ No Limelight data");
            telemetry.update();
            return;
        }
        
        if (!result.isValid()) {
            telemetry.addData("Status", "⚠️ Limelight data invalid");
            telemetry.update();
            return;
        }
        
        // Get AprilTag fiducial results
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        
        telemetry.addLine("=== LIMELIGHT STATUS ===");
        telemetry.addData("Valid", result.isValid() ? "✓ YES" : "✗ NO");
        telemetry.addData("AprilTags Detected", fiducialResults.size());
        telemetry.addData("", "");
        
        if (fiducialResults.isEmpty()) {
            telemetry.addData("Status", "❌ No AprilTags visible");
            telemetry.addLine("Point camera at an AprilTag!");
            telemetry.update();
            return;
        }
        
        // Find target tag or use first one
        LLResultTypes.FiducialResult targetFiducial = null;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (TARGET_APRILTAG_ID == -1 || fr.getFiducialId() == TARGET_APRILTAG_ID) {
                targetFiducial = fr;
                break;
            }
        }
        
        if (targetFiducial == null) {
            telemetry.addData("Status", "❌ Target tag not found");
            telemetry.addData("Looking for ID", TARGET_APRILTAG_ID);
            telemetry.addLine("Available tags:");
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("  Tag", fr.getFiducialId());
            }
            telemetry.update();
            return;
        }
        
        // Get angles from Limelight
        double tx = targetFiducial.getTargetXDegrees();  // Horizontal angle
        double ty = targetFiducial.getTargetYDegrees();  // Vertical angle
        int tagId = targetFiducial.getFiducialId();
        
        telemetry.addLine("=== TARGET APRILTAG ===");
        telemetry.addData("Tag ID", tagId);
        telemetry.addData("tx (Horizontal °)", "%.2f", tx);
        telemetry.addData("ty (Vertical °)", "%.2f", ty);
        telemetry.addData("", "");
        
        // Calculate total angle to target accounting for camera mount angle
        double angleToTarget = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
        
        telemetry.addLine("=== DISTANCE CALCULATION ===");
        telemetry.addData("Mount Angle", "%.2f°", LIMELIGHT_MOUNT_ANGLE_DEG);
        telemetry.addData("+ Vertical Angle (ty)", "%.2f°", ty);
        telemetry.addData("= Total Angle", "%.2f°", angleToTarget);
        telemetry.addData("Height Difference", "%.2f inches", heightDifference);
        telemetry.addData("", "");
        
        // Check if angle is valid (avoid division by zero)
        if (Math.abs(angleToTarget) < 0.5) {
            telemetry.addData("Status", "⚠️ Angle too small for accurate calculation");
            telemetry.addData("Total angle", "%.2f° (need > 0.5°)", angleToTarget);
            telemetry.update();
            return;
        }
        
        // ========== TRIGONOMETRY CALCULATION ==========
        // Using: distance = height / tan(angle)
        double horizontalDistance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
        
        // Calculate diagonal distance (direct line from camera to tag center)
        double diagonalDistance = Math.sqrt(
            horizontalDistance * horizontalDistance + 
            heightDifference * heightDifference
        );
        
        // Calculate horizontal offset (left/right from center)
        double xOffset = horizontalDistance * Math.tan(Math.toRadians(tx));
        
        telemetry.addLine("=== ✓ CALCULATED DISTANCES ===");
        telemetry.addData("Horizontal Distance", "%.2f inches", horizontalDistance);
        telemetry.addData("Horizontal Distance", "%.2f feet", horizontalDistance / 12.0);
        telemetry.addData("Diagonal Distance", "%.2f inches", diagonalDistance);
        telemetry.addData("X Offset (left/right)", "%.2f inches", xOffset);
        telemetry.addData("", "");
        
        // ========== LIMELIGHT POSE ESTIMATION (Alternative Method) ==========
        telemetry.addLine("=== LIMELIGHT POSE3D (Alternative) ===");
        Pose3D robotPose = targetFiducial.getRobotPoseTargetSpace();
        if (robotPose != null && robotPose.getPosition() != null) {
            double poseX = robotPose.getPosition().x;
            double poseY = robotPose.getPosition().y;
            double poseZ = robotPose.getPosition().z;
            
            double poseHorizontalDist = Math.sqrt(poseX * poseX + poseY * poseY);
            double poseDiagonalDist = Math.sqrt(poseX * poseX + poseY * poseY + poseZ * poseZ);
            
            telemetry.addData("Pose X", "%.2f inches", poseX);
            telemetry.addData("Pose Y", "%.2f inches", poseY);
            telemetry.addData("Pose Z", "%.2f inches", poseZ);
            telemetry.addData("Pose Horizontal Dist", "%.2f inches", poseHorizontalDist);
            telemetry.addData("Pose Horizontal Dist", "%.2f feet", poseHorizontalDist / 12.0);
            telemetry.addData("Pose Diagonal Dist", "%.2f inches", poseDiagonalDist);
            telemetry.addData("", "");
            
            // Compare methods
            telemetry.addLine("=== METHOD COMPARISON ===");
            double difference = Math.abs(horizontalDistance - poseHorizontalDist);
            telemetry.addData("Trig Method", "%.2f inches", horizontalDistance);
            telemetry.addData("Pose Method", "%.2f inches", poseHorizontalDist);
            telemetry.addData("Difference", "%.2f inches", difference);
            
            if (difference < 3.0) {
                telemetry.addData("Agreement", "✓ GOOD (< 3 inches)");
            } else if (difference < 6.0) {
                telemetry.addData("Agreement", "⚠️ OK (3-6 inches)");
            } else {
                telemetry.addData("Agreement", "❌ POOR (> 6 inches)");
            }
        } else {
            telemetry.addData("Pose3D", "Not available");
        }
        
        telemetry.addData("", "");
        telemetry.addLine("=== CONSTANTS (Adjust on Dashboard) ===");
        telemetry.addData("APRILTAG_HEIGHT_INCHES", "%.1f", APRILTAG_HEIGHT_INCHES);
        telemetry.addData("LIMELIGHT_HEIGHT_INCHES", "%.1f", LIMELIGHT_HEIGHT_INCHES);
        telemetry.addData("LIMELIGHT_MOUNT_ANGLE_DEG", "%.1f", LIMELIGHT_MOUNT_ANGLE_DEG);
        telemetry.addData("TARGET_APRILTAG_ID", "%d", TARGET_APRILTAG_ID);
        
        telemetry.update();
    }
    
    @Override
    public void stop() {
        limelight.stop();
    }
}

