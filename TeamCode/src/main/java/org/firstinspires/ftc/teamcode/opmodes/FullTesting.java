package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;

import java.util.List;

//import pedropathing.constants.*;

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot
 * on FTC Dashboard (192/168/43/1:8080/dash). You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config
@TeleOp(name = "Full Testing")
public class FullTesting extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;

    private DcMotorEx fl, fr, bl, br;

    private Servo turret1;
    private Servo turret2;

    private Limelight3A limelight;
    
    // Shooter hardware
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1;
    private PIDFController shooterPID;

    // Target coordinates for distance calculation
    public static double targetX = 128.0;
    public static double targetY = 128.0;
    
    // Goal zone coordinates (for RPM calculation)
    public static double goalZoneX = 115.0; // Goal zone X coordinate in inches
    public static double goalZoneY = 121.0; // Goal zone Y coordinate in inches
    
    // Height measurements
    public static double aprilTagHeight = 30.0; // AprilTag height in inches
    public static double limelightHeight = 13.5; // Limelight height in inches
    public static double heightDifference = aprilTagHeight - limelightHeight; // 16.3 inches
    public static double limelightMountAngle = 19.0; // Limelight mount angle in degrees
    
    // Turret servo constants
    public static double turretCenterPosition = 0.51; // Servo position for 0 degrees
    public static double turretLeftPosition = 0.275; // Servo position for max left
    public static double turretRightPosition = 0.745; // Servo position for max right
    public static double turretMaxAngle = 90.0; // Max angle in degrees (left or right from center)
    
    // Shooter PIDF Constants - From VelocityFinder
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;
    
    // Short range PIDF (< 6 feet)
    public static double p = 0.002;
    public static double i = 0.0;
    public static double d = 0.0001;
    public static double f = 0.00084;
    public static double kV = 0.0008;
    public static double kS = 0.01;
    public static double I_ZONE = 250.0;
    public static double hood1Position = 0.54;
    
    // Long range PIDF (>= 6 feet)
    public static double pLong = 0.01;
    public static double iLong = 0.0;
    public static double dLong = 0.0001;
    public static double fLong = 0.00084;
    public static double kVLong = 0.0008;
    public static double kSLong = 0.01;
    public static double I_ZONE_LONG = 250.0;
    public static double hood1PositionLong = 0.45;
    
    // Linear regression for RPM calculation: RPM = 100 * (feet_from_goal) + 1150
    // where x is feet from goal zone, y is RPM
    // Formula: y = 100x + 1150
    private static final double RPM_SLOPE = 100.0;  // m in y = mx + b
    private static final double RPM_INTERCEPT = 1150.0;  // b in y = mx + b
    
    // Auto-shoot state machine
    private boolean lastA = false;
    private boolean lastLeftTrigger = false;
    private boolean shooting = false;
    private boolean autoTransfer = false;
    private int shootState = 0;
    private int transferState = 0;
    private ElapsedTime shootTimer;
    private ElapsedTime transferTimer;
    
    // RPM tolerance for "at target" detection
    public static double RPM_TOLERANCE = 100.0;
    
    // Voltage compensation (always on)
    private static final double NOMINAL_VOLTAGE = 12.0;

    /**
     * This initializes the PoseUpdater, the mecanum drive motors, and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        
        // Initialize drive motors
        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Initialize turret servos
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        
        // Initialize shooter hardware
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");
        
        reargate = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        hood1 = hardwareMap.get(Servo.class, "hood 1");
        
        // Set shooter motor directions
        shootl.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeback.setDirection(DcMotorSimple.Direction.REVERSE);
        intakefront.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Setup all motors
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        // Initialize shooter PIDF controller
        shooterPID = new PIDFController(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);
        
        // Initialize auto-shoot timers
        shootTimer = new ElapsedTime();
        transferTimer = new ElapsedTime();
        
        // Set initial servo positions
        launchgate.setPosition(0.5);
        hood1.setPosition(hood1Position);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.setMsTransmissionInterval(11);
        
        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
            telemetryA.addLine("Limelight initialized successfully");
        } catch (Exception e) {
            telemetryA.addLine("Warning: Limelight not found - " + e.getMessage());
            limelight = null;
        }
        
        telemetryA.addLine("Full Testing OpMode - Localization + Shooter");
        telemetryA.addLine("Gamepad 1 Controls:");
        telemetryA.addLine("  - Right Stick: Mecanum drive (Y/X)");
        telemetryA.addLine("  - Left Stick X: Rotation");
        telemetryA.addLine("  - Left Bumper: Back intake");
        telemetryA.addLine("  - Right Bumper: Front intake");
        telemetryA.addLine("  - Right Trigger: Spin up shooter (RPM auto-calculated)");
        telemetryA.addLine("  - Left Trigger: Auto-transfer 3 balls when at speed");
        telemetryA.addLine("  - A Button: Auto-shoot 3 balls (distance-based)");
        telemetryA.addLine("RPM Formula: RPM = 100 * (feet from goal) + 1150");
        telemetryA.addLine("Auto-transfer adapts to distance (fast <7ft, slow ≥7ft)");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the FTC
     * Dashboard telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        poseUpdater.update();
        dashboardPoseTracker.update();

        // Drive controls - matching VelocityFinder setup
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x * 1.1;
        double rx = gamepad1.left_stick_x;
        
        double scale = 1;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        
        double flPower = (y + x + rx) / denominator * scale;
        double blPower = (y - x + rx) / denominator * scale;
        double frPower = (y - x - rx) / denominator * scale;
        double brPower = (y + x - rx) / denominator * scale;
        
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        // Get current position
        double currentX = poseUpdater.getPose().getX();
        double currentY = poseUpdater.getPose().getY();

        // Calculate distance to target (119, 119)
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // Calculate angle to target relative to field (0 degrees = east, 90 = north)
        double angleToTargetField = Math.atan2(deltaY, deltaX);
        
        // Calculate angle relative to robot (turret angle needed)
        // Subtract robot heading to get relative angle
        double currentHeading = poseUpdater.getPose().getHeading();
        double turretAngle = angleToTargetField - currentHeading;
        
        // Normalize angle to [-PI, PI]
        while (turretAngle > Math.PI) turretAngle -= 2 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;
        
        // Convert to degrees
        double turretAngleDegrees = Math.toDegrees(turretAngle);
        
        // Calculate servo position
        // Clamp angle to valid range
        double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDegrees));
        
        // Convert angle to servo position (FLIPPED)
        double servoPosition;
        if (clampedAngle >= 0) {
            // Positive angle = turn right
            double servoRange = turretRightPosition - turretCenterPosition; // 0.235
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            // Negative angle = turn left
            double servoRange = turretCenterPosition - turretLeftPosition; // 0.235
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }
        
        // Set servo positions
        turret1.setPosition(servoPosition);
        turret2.setPosition(servoPosition);
        
        // ===== SHOOTER CONTROL =====
        // Intake controls - split between bumpers
        // Left bumper: back intake
        if (gamepad1.left_bumper) {
            intakeback.setPower(1.0);
        } else {
            intakeback.setPower(0);
        }
        
        // Right bumper: front intake
        if (gamepad1.right_bumper) {
            intakefront.setPower(1.0);
        } else {
            intakefront.setPower(0);
        }
        
        // ==================== AUTO-SHOOT CONTROL ====================
        // Detect A button press for auto-shoot sequence
        boolean currentA = gamepad1.a;
        if (currentA && !lastA && !shooting) {
            // Start 3-ball shooting sequence
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }
        lastA = currentA;
        
        // Run shooting state machine (3 shots)
        String shootStatus = "Ready";
        if (shooting) {
            switch (shootState) {
                case 0: // Spin up shooter
                    shootStatus = "Spinning up...";
                    if (shootTimer.seconds() > 1.0) {
                        shootState = 1;
                        shootTimer.reset();
                    }
                    break;
                    
                case 1: // Start intakes
                    shootStatus = "Starting intakes...";
                    intakefront.setPower(-1.0);
                    intakeback.setPower(-1.0);
                    if (shootTimer.seconds() > 0.1) {
                        shootState = 2;
                        shootTimer.reset();
                    }
                    break;
                    
                case 2: // Fire shot 1
                    shootStatus = "Firing shot 1/3";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2) {
                        shootState = 3;
                        shootTimer.reset();
                    }
                    break;
                    
                case 3: // Reset gate 1
                    shootStatus = "Reset 1/3";
                    launchgate.setPosition(0.5);
                    if (shootTimer.seconds() > 0.3) {
                        shootState = 4;
                        shootTimer.reset();
                    }
                    break;
                    
                case 4: // Fire shot 2
                    shootStatus = "Firing shot 2/3";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2) {
                        shootState = 5;
                        shootTimer.reset();
                    }
                    break;
                    
                case 5: // Reset gate 2
                    shootStatus = "Reset 2/3";
                    launchgate.setPosition(0.5);
                    if (shootTimer.seconds() > 0.3) {
                        shootState = 6;
                        shootTimer.reset();
                    }
                    break;
                    
                case 6: // Fire shot 3
                    shootStatus = "Firing shot 3/3";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2) {
                        shootState = 7;
                        shootTimer.reset();
                    }
                    break;
                    
                case 7: // Reset gate 3 and stop
                    shootStatus = "Complete!";
                    launchgate.setPosition(0.5);
                    intakefront.setPower(0);
                    intakeback.setPower(0);
                    if (shootTimer.seconds() > 0.2) {
                        shooting = false;
                        shootState = 0;
                    }
                    break;
            }
        }
        
        // Shooter velocity control - Hold Right Trigger OR auto-shoot active
        boolean shooterOn = gamepad1.right_trigger > 0.1 || shooting;
        
        // Calculate distance to goal zone for RPM calculation
        double deltaGoalX = goalZoneX - currentX;
        double deltaGoalY = goalZoneY - currentY;
        double distanceToGoalInches = Math.sqrt(deltaGoalX * deltaGoalX + deltaGoalY * deltaGoalY);
        double distanceToGoalFeet = distanceToGoalInches / 12.0;  // Convert inches to feet
        
        // Calculate target RPM using linear regression: RPM = 100 * (feet) + 1150
        // Calculate target RPM: use formula up to 7 feet, then cap at 2100 RPM
        double calculatedTargetRPM;
        if (distanceToGoalFeet >= 7.0) {
            calculatedTargetRPM = 2100.0;  // Fixed RPM for 7+ feet
        } else {
            calculatedTargetRPM = RPM_SLOPE * distanceToGoalFeet + RPM_INTERCEPT;
            calculatedTargetRPM = Math.max(1250.0, Math.min(2100.0, calculatedTargetRPM));
        }
        
        // Determine if we're shooting long range (>= 6 feet)
        boolean isLongRange = distanceToGoalFeet >= 6.0;
        
        // Use appropriate PIDF values based on distance
        double currentP = isLongRange ? pLong : p;
        double currentI = isLongRange ? iLong : i;
        double currentD = isLongRange ? dLong : d;
        double currentF = isLongRange ? fLong : f;
        double currentKV = isLongRange ? kVLong : kV;
        double currentKS = isLongRange ? kSLong : kS;
        double currentIZone = isLongRange ? I_ZONE_LONG : I_ZONE;
        double currentHood = isLongRange ? hood1PositionLong : hood1Position;
        
        // Update PIDF coefficients
        shooterPID.setPIDF(currentP, currentI, currentD, currentF);
        shooterPID.setIntegrationBounds(-currentIZone, currentIZone);
        
        // Set hood position based on distance
        hood1.setPosition(currentHood);
        
        // Convert target RPM to ticks per second
        double targetTPS = rpmToTicksPerSec(calculatedTargetRPM);
        
        // Read current velocities
        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double vAvg = 0.5 * (vR + vL);
        
        // Convert to RPM for display
        double shootrVelocityRPM = ticksPerSecToRPM(vR);
        double shootlVelocityRPM = ticksPerSecToRPM(vL);
        double avgVelocityRPM = ticksPerSecToRPM(vAvg);
        
        double shooterPower = 0;
        double pidfOutput = 0;
        double additionalFF = 0;
        
        if (shooterOn) {
            // PIDF control (F term is built-in and multiplied by setpoint)
            pidfOutput = shooterPID.calculate(vAvg, targetTPS);
            
            // Additional feedforward using kV and kS (use current values based on distance)
            double sgn = Math.signum(targetTPS);
            additionalFF = (Math.abs(targetTPS) > 1e-6) ? (currentKS * sgn + currentKV * targetTPS) : 0.0;
            
            // Total power (PIDF output already includes F*setpoint)
            // If using only F term, set kS and kV to 0
            shooterPower = pidfOutput + additionalFF;
            
            // Safety: prevent overshoot
            if (avgVelocityRPM >= calculatedTargetRPM && shooterPower > 0) {
                shooterPower = Math.min(shooterPower, 0.5);
            }
            
            // Clamp
            shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
            
            // Voltage compensation (always on): power * (currentVoltage / 12V)
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double compensatedPower = shooterPower * (voltage / NOMINAL_VOLTAGE);
            compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));
            
            shootr.setPower(compensatedPower);
            shootl.setPower(compensatedPower);
        } else {
            shootr.setPower(0);
            shootl.setPower(0);
            shooterPID.reset();
        }
        
        // ==================== AUTO-TRANSFER CONTROL (Left Trigger) ====================
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.1;
        
        // Check if shooter is at target velocity
        boolean atTargetSpeed = Math.abs(avgVelocityRPM - calculatedTargetRPM) < RPM_TOLERANCE;
        
        // Detect left trigger press for auto-transfer
        if (currentLeftTrigger && !lastLeftTrigger && !autoTransfer && !shooting && shooterOn && atTargetSpeed) {
            // Start auto-transfer sequence
            autoTransfer = true;
            transferState = 0;
            transferTimer.reset();
        }
        lastLeftTrigger = currentLeftTrigger;
        
        // Run auto-transfer state machine
        String transferStatus = "Ready";
        if (autoTransfer) {
            boolean isLongRangeTransfer = distanceToGoalFeet >= 7.0;
            
            if (isLongRangeTransfer) {
                // LONG RANGE: Intakes on, push, intakes off, wait 0.3s, repeat 3x
                switch (transferState) {
                    case 0: // Shot 1: Intakes on
                        transferStatus = "Shot 1/3: Intakes on";
                        intakefront.setPower(-1.0);
                        intakeback.setPower(-1.0);
                        if (transferTimer.seconds() > 0.1) {
                            transferState = 1;
                            transferTimer.reset();
                        }
                        break;
                    case 1: // Shot 1: Push
                        transferStatus = "Shot 1/3: Pushing";
                        launchgate.setPosition(0.8);
                        if (transferTimer.seconds() > 0.2) {
                            transferState = 2;
                            transferTimer.reset();
                        }
                        break;
                    case 2: // Shot 1: Reset and intakes off
                        transferStatus = "Shot 1/3: Reset";
                        launchgate.setPosition(0.5);
                        intakefront.setPower(0);
                        intakeback.setPower(0);
                        if (transferTimer.seconds() > 0.3) {
                            transferState = 3;
                            transferTimer.reset();
                        }
                        break;
                    case 3: // Shot 2: Intakes on
                        transferStatus = "Shot 2/3: Intakes on";
                        intakefront.setPower(-1.0);
                        intakeback.setPower(-1.0);
                        if (transferTimer.seconds() > 0.1) {
                            transferState = 4;
                            transferTimer.reset();
                        }
                        break;
                    case 4: // Shot 2: Push
                        transferStatus = "Shot 2/3: Pushing";
                        launchgate.setPosition(0.8);
                        if (transferTimer.seconds() > 0.2) {
                            transferState = 5;
                            transferTimer.reset();
                        }
                        break;
                    case 5: // Shot 2: Reset and intakes off
                        transferStatus = "Shot 2/3: Reset";
                        launchgate.setPosition(0.5);
                        intakefront.setPower(0);
                        intakeback.setPower(0);
                        if (transferTimer.seconds() > 0.3) {
                            transferState = 6;
                            transferTimer.reset();
                        }
                        break;
                    case 6: // Shot 3: Intakes on
                        transferStatus = "Shot 3/3: Intakes on";
                        intakefront.setPower(-1.0);
                        intakeback.setPower(-1.0);
                        if (transferTimer.seconds() > 0.1) {
                            transferState = 7;
                            transferTimer.reset();
                        }
                        break;
                    case 7: // Shot 3: Push
                        transferStatus = "Shot 3/3: Pushing";
            launchgate.setPosition(0.8);
                        if (transferTimer.seconds() > 0.2) {
                            transferState = 8;
                            transferTimer.reset();
                        }
                        break;
                    case 8: // Shot 3: Final reset
                        transferStatus = "Complete!";
                        launchgate.setPosition(0.5);
                        intakefront.setPower(0);
                        intakeback.setPower(0);
                        if (transferTimer.seconds() > 0.2) {
                            autoTransfer = false;
                            transferState = 0;
                        }
                        break;
                }
        } else {
                // SHORT RANGE: Run intakes continuously, push 3x rapidly
                switch (transferState) {
                    case 0: // Start intakes
                        transferStatus = "Starting intakes...";
                        intakefront.setPower(-1.0);
                        intakeback.setPower(-1.0);
                        if (transferTimer.seconds() > 0.1) {
                            transferState = 1;
                            transferTimer.reset();
                        }
                        break;
                    case 1: // Push 1
                        transferStatus = "Firing 1/3";
                        launchgate.setPosition(0.8);
                        if (transferTimer.seconds() > 0.2) {
                            transferState = 2;
                            transferTimer.reset();
                        }
                        break;
                    case 2: // Reset 1
                        transferStatus = "Reset 1/3";
                        launchgate.setPosition(0.5);
                        if (transferTimer.seconds() > 0.3) {
                            transferState = 3;
                            transferTimer.reset();
                        }
                        break;
                    case 3: // Push 2
                        transferStatus = "Firing 2/3";
                        launchgate.setPosition(0.8);
                        if (transferTimer.seconds() > 0.2) {
                            transferState = 4;
                            transferTimer.reset();
                        }
                        break;
                    case 4: // Reset 2
                        transferStatus = "Reset 2/3";
                        launchgate.setPosition(0.5);
                        if (transferTimer.seconds() > 0.3) {
                            transferState = 5;
                            transferTimer.reset();
                        }
                        break;
                    case 5: // Push 3
                        transferStatus = "Firing 3/3";
                        launchgate.setPosition(0.8);
                        if (transferTimer.seconds() > 0.2) {
                            transferState = 6;
                            transferTimer.reset();
                        }
                        break;
                    case 6: // Final reset and stop intakes
                        transferStatus = "Complete!";
                        launchgate.setPosition(0.5);
                        intakefront.setPower(0);
                        intakeback.setPower(0);
                        if (transferTimer.seconds() > 0.2) {
                            autoTransfer = false;
                            transferState = 0;
                        }
                        break;
                }
            }
        } else if (!currentLeftTrigger && !autoTransfer) {
            // Manual launch gate control when not auto-transferring
            launchgate.setPosition(0.5);
        }


        telemetryA.addData("x", currentX);
        telemetryA.addData("y", currentY);
        telemetryA.addData("heading (rad)", currentHeading);
        telemetryA.addData("heading (deg)", Math.toDegrees(currentHeading));
        telemetryA.addData("", ""); // Empty line
        telemetryA.addData("Distance to Target", "%.2f inches", distance);
        telemetryA.addData("Turret Angle", "%.2f degrees", turretAngleDegrees);
        telemetryA.addData("Turret Servo Position", "%.3f", servoPosition);
        if (turretAngleDegrees < -turretMaxAngle || turretAngleDegrees > turretMaxAngle) {
            telemetryA.addData("WARNING", "Target out of turret range!");
        }
        
        // Add shooter telemetry
        telemetryA.addData("", ""); // Empty line
        telemetryA.addLine("=== Shooter Status ===");
        telemetryA.addData("Shooter", shooterOn ? "RUNNING" : "STOPPED");
        telemetryA.addData("At Target Speed", atTargetSpeed ? "✓ YES" : "NO");
        if (shooting) {
            telemetryA.addData("Auto-Shoot", shootStatus);
            telemetryA.addData("State", shootState);
        }
        if (autoTransfer) {
            telemetryA.addData("Auto-Transfer", transferStatus);
            telemetryA.addData("Transfer State", transferState);
            telemetryA.addData("Transfer Mode", distanceToGoalFeet >= 7.0 ? "LONG (slow)" : "SHORT (fast)");
        }
        telemetryA.addData("Distance Range", isLongRange ? "LONG (≥6ft) - p=0.01, hood=0.45" : "SHORT (<6ft) - p=0.002, hood=0.54");
        telemetryA.addData("Distance to Goal", "%.2f inches (%.2f feet)", distanceToGoalInches, distanceToGoalFeet);
        telemetryA.addData("Calculated Target RPM", "%.0f (RPM = 100*%.2f + 1150)", calculatedTargetRPM, distanceToGoalFeet);
        telemetryA.addData("Current RPM", "%.0f", avgVelocityRPM);
        telemetryA.addData("Right Motor RPM", "%.0f", shootrVelocityRPM);
        telemetryA.addData("Left Motor RPM", "%.0f", shootlVelocityRPM);
        telemetryA.addData("Error (RPM)", "%.0f", calculatedTargetRPM - avgVelocityRPM);
        
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double compensationFactor = currentVoltage / NOMINAL_VOLTAGE;
        
        telemetryA.addData("Battery Voltage", "%.2f V", currentVoltage);
        telemetryA.addData("Voltage Comp", "×%.3f", compensationFactor);
        telemetryA.addData("Shooter Power (calc)", "%.3f", shooterPower);
        
        if (shooterOn) {
            double compensatedPower = shooterPower * compensationFactor;
            telemetryA.addData("Actual Motor Power", "%.3f", compensatedPower);
        }
        
        if (shooterOn) {
            telemetryA.addData("PIDF Output", "%.4f", pidfOutput);
            telemetryA.addData("Additional FF", "%.4f (kS=%.4f + kV*TPS=%.4f)", 
                additionalFF, currentKS * Math.signum(targetTPS), currentKV * targetTPS);
            telemetryA.addData("F Term Contribution", "%.4f (F*setpoint = %.4f*%.1f)", 
                currentF * targetTPS, currentF, targetTPS);
        }
        telemetryA.addData("Front Intake", gamepad1.right_bumper ? "RUNNING" : "STOPPED");
        telemetryA.addData("Back Intake", gamepad1.left_bumper ? "RUNNING" : "STOPPED");
        telemetryA.addData("Launch Gate", gamepad1.left_trigger > 0.1 ? "FIRING" : "RESET");
        telemetryA.addData("Hood Position", "%.2f", hood1Position);
        telemetryA.addData("", "");
        telemetryA.addLine("=== PIDF Tuning ===");
        telemetryA.addData("P", "%.6f", p);
        telemetryA.addData("I", "%.6f", i);
        telemetryA.addData("D", "%.6f", d);
        telemetryA.addData("F", "%.6f (NEW - tune this!)", f);
        telemetryA.addData("kV", "%.6f (optional, can set to 0)", kV);
        telemetryA.addData("kS", "%.6f (optional, can set to 0)", kS);
        
        // Add Limelight data
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetryA.addData("", ""); // Empty line
                telemetryA.addLine("=== Limelight Data ===");
                telemetryA.addData("tx (degrees)", "%.2f", result.getTx());
                telemetryA.addData("ty (degrees)", "%.2f", result.getTy());
                
                // Display AprilTag fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (!fiducialResults.isEmpty()) {
                    telemetryA.addData("AprilTags Detected", fiducialResults.size());
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        double tx = fr.getTargetXDegrees();
                        double ty = fr.getTargetYDegrees();
                        
                        telemetryA.addData("  Tag ID", "%d", fr.getFiducialId());
                        telemetryA.addData("    X (degrees)", "%.2f", tx);
                        telemetryA.addData("    Y (degrees)", "%.2f", ty);
                        
                        // Calculate distance using height difference, mount angle, and vertical angle
                        double angleToTarget = limelightMountAngle + ty;
                        
                        if (Math.abs(angleToTarget) > 0.5) { // Only calculate if we have a valid angle
                            // Horizontal distance calculation accounting for mount angle
                            double horizontalDistance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
                            
                            // Diagonal distance from lens to center of AprilTag
                            double diagonalDistance = Math.sqrt(
                                horizontalDistance * horizontalDistance + 
                                heightDifference * heightDifference
                            );
                            
                            // Convert horizontal angle to inches offset
                            double x_inches = horizontalDistance * Math.tan(Math.toRadians(tx));
                            
                            telemetryA.addData("    Angle to Target", "%.2f deg", angleToTarget);
                            telemetryA.addData("    Horizontal Dist", "%.2f inches", horizontalDistance);
                            telemetryA.addData("    Diagonal Dist", "%.2f inches", diagonalDistance);
                            telemetryA.addData("    X Offset", "%.2f inches", x_inches);
                            telemetryA.addData("    Height Diff", "%.2f inches", heightDifference);
                        } else {
                            telemetryA.addData("    Distance", "Invalid angle");
                        }
                        
                        // Also show the 3D pose data if available
                        org.firstinspires.ftc.robotcore.external.navigation.Pose3D targetPose = fr.getRobotPoseTargetSpace();
                        if (targetPose != null && targetPose.getPosition() != null) {
                            telemetryA.addData("    Pose X", "%.2f", targetPose.getPosition().x);
                            telemetryA.addData("    Pose Y", "%.2f", targetPose.getPosition().y);
                            telemetryA.addData("    Pose Z", "%.2f", targetPose.getPosition().z);
                        }
                    }
                } else {
                    telemetryA.addData("AprilTags", "None detected");
                }
            } else {
                telemetryA.addData("", ""); // Empty line
                telemetryA.addData("Limelight", "No valid data");
            }
        }
        
        telemetryA.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
    
    /**
     * Convert RPM to ticks per second
     */
    private static double rpmToTicksPerSec(double rpm) {
        double motorRPM = rpm * GEAR_RATIO;
        return (motorRPM / 60.0) * TICKS_PER_REV;
    }

    /**
     * Convert ticks per second to RPM
     */
    private static double ticksPerSecToRPM(double tps) {
        double motorRPM = (tps / TICKS_PER_REV) * 60.0;
        return motorRPM / GEAR_RATIO;
    }
    
    @Override
    public void stop() {
        // Stop all motors on OpMode stop
        shootr.setPower(0);
        shootl.setPower(0);
        intakefront.setPower(0);
        intakeback.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}

