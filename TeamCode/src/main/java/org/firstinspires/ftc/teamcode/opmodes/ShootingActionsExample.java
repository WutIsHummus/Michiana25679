package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * ============================================================================
 * SHOOTING ACTIONS - ALL THE WAYS TO USE THEM
 * ============================================================================
 * 
 * METHOD 1: Direct coordinates
 *   actions.threeBallFromPosition(60.0, 80.0, Math.toRadians(45));
 * 
 * METHOD 2: From Follower (in PathChain callback)
 *   .addParametricCallback(0.5, actions.threeBallFromFollower(follower))
 * 
 * METHOD 3: From PoseUpdater
 *   actions.threeBallFromPose(poseUpdater);
 * 
 * METHOD 4: Manual parameters
 *   actions.aimAndShoot(1500, 0.54, 45.0);  // RPM, hood, turret angle
 * 
 * ALL methods automatically:
 * - Calculate RPM using regression
 * - Aim turret
 * - Set hood
 * - Use PID control
 * - Apply voltage compensation
 * ============================================================================
 */
@Autonomous(name = "Shooting Actions Examples")
public class ShootingActionsExample extends OpMode {
    
    private Follower follower;
    private RobotActions actions;
    
    private DcMotorEx intakefront, intakeback, shootr, shootl;
    private Servo launchgate, reargate, hood1, turret1, turret2;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        
        // Initialize hardware
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");
        
        shootr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        reargate = hardwareMap.get(Servo.class, "reargate");
        hood1 = hardwareMap.get(Servo.class, "hood1");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        
        // Create RobotActions
        actions = new RobotActions(
            intakefront, intakeback,
            shootr, shootl,
            launchgate, reargate,
            hood1, turret1, turret2,
            voltageSensor
        );
        
        follower.setStartingPose(new Pose(10, 10, 0));
        
        // ============================================================================
        // EXAMPLE 1: Using in PathChain (shoot after path completes)
        // ============================================================================
        // Note: .addParametricCallback() expects Runnable, not Action
        // So you use it in your loop after path completes (see AutoShootRegression.java)
        
        telemetry.addLine("✓ Examples ready!");
        telemetry.addLine("");
        telemetry.addLine("╔════════════════════════════════════════╗");
        telemetry.addLine("║  REGRESSION SHOOTING METHODS           ║");
        telemetry.addLine("╚════════════════════════════════════════╝");
        telemetry.addLine("");
        telemetry.addLine("SPIN UP ONLY (no aiming/shooting):");
        telemetry.addLine("  actions.spinUpForDistance(5.5)");
        telemetry.addLine("  actions.spinUpFromFollower(follower)");
        telemetry.addLine("  actions.spinUpFromPosition(x, y)");
        telemetry.addLine("");
        telemetry.addLine("THREE-BALL AUTO:");
        telemetry.addLine("  actions.threeBallFromPosition(");
        telemetry.addLine("    60.0, 80.0, Math.toRadians(45))");
        telemetry.addLine("  actions.threeBallFromFollower(follower)");
        telemetry.addLine("");
        telemetry.addLine("MANUAL:");
        telemetry.addLine("  actions.aimAndShoot(1500, 0.54, 0)");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        
        Pose pose = follower.getPose();
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Status", follower.isBusy() ? "Following path" : "Idle");
        telemetry.update();
    }
}

