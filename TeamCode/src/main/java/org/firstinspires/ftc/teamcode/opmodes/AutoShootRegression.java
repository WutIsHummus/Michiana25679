package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierPoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * AUTONOMOUS REGRESSION SHOOTING EXAMPLE
 * 
 * This shows how to use RobotActions' regression-based shooting in autonomous:
 * 
 * KEY FEATURES:
 * 1. Drive to position
 * 2. Auto-shoot with regression (calculates RPM, aims turret)
 * 3. Continue path
 * 
 * USAGE IN PATHCHAIN:
 * .addParametricCallback(0.5, actions.threeBallFromFollower(follower))
 */
@Config
@Disabled
@Autonomous(name = "Auto Shoot Regression Example")
public class AutoShootRegression extends OpMode {
    
    private Follower follower;
    private RobotActions actions;
    
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo launchgate, reargate, hood1, turret1, turret2;
    
    private PathChain shootingPath;
    private Action currentAction = null;
    private boolean shootingActionStarted = false;

    @Override
    public void init() {
        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        
        // Initialize shooter hardware
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");
        
        shootr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        reargate = hardwareMap.get(Servo.class, "reargate");
        hood1 = hardwareMap.get(Servo.class, "hood1");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        
        // ========================================
        // CREATE ROBOT ACTIONS
        // ========================================
        actions = new RobotActions(
            intakefront, intakeback,
            shootr, shootl,
            launchgate, reargate,
            hood1, turret1, turret2,
            voltageSensor
        );
        
        // Set starting pose
        follower.setStartingPose(new Pose(10, 10, Math.toRadians(0)));
        
        // Build path - drive to shooting position
        shootingPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(10, 10, 0),
                        new Pose(30, 50, 0),
                        new Pose(60, 80, 0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
        
        telemetry.addLine("✓ Ready to shoot!");
        telemetry.addLine("");
        telemetry.addLine("Autonomous sequence:");
        telemetry.addLine("1. Drive to shooting position");
        telemetry.addLine("2. Auto-shoot 3 balls (regression + PID)");
        telemetry.addLine("3. Report results");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start following path
        follower.followPath(shootingPath);
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();
        
        // Check if path is complete and shooting hasn't started
        if (!follower.isBusy() && !shootingActionStarted) {
            telemetry.addLine("🎯 Path complete! Starting AUTO SHOOT...");
            telemetry.update();
            
            // ========================================
            // DISTANCE-BASED SHOOTING - SUPER EASY!
            // ========================================
            // Just say "shoot 48 inches straight ahead"!
            // (48, 0) = 48 inches at 0 degrees (straight)
            // ========================================
            currentAction = actions.threeBallAtDistance(48, 0, follower);
            
            // OR use goal-based shooting:
            // currentAction = actions.threeBallFromFollower(follower);
            
            shootingActionStarted = true;
        }
        
        // Run the shooting action if it exists
        if (currentAction != null) {
            boolean actionComplete = currentAction.run(new com.acmerobotics.dashboard.telemetry.TelemetryPacket());
            
            if (actionComplete) {
                telemetry.addLine("✓ SHOOTING COMPLETE!");
                telemetry.addLine("");
                telemetry.addLine("Autonomous finished successfully.");
                currentAction = null;
            }
        }
        
        // Display status
        Pose currentPose = follower.getPose();
        telemetry.addData("", "");
        telemetry.addLine("=== STATUS ===");
        telemetry.addData("Path Following", follower.isBusy() ? "ACTIVE" : "COMPLETE");
        telemetry.addData("Shooting", shootingActionStarted ? 
            (currentAction != null ? "IN PROGRESS" : "COMPLETE") : "WAITING");
        
        telemetry.addData("", "");
        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X", "%.2f inches", currentPose.getX());
        telemetry.addData("Y", "%.2f inches", currentPose.getY());
        telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(currentPose.getHeading()));
        
        telemetry.update();
    }
}

