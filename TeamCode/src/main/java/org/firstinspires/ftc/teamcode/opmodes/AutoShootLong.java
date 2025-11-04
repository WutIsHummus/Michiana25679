package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
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

@Config
@TeleOp(name = "Auto Shoot Long Range")
public class AutoShootLong extends OpMode {
    
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Limelight3A limelight;
    
    private PIDFController shooterPID;
    private ElapsedTime shootTimer;
    
    // Goal zone coordinates (adjustable from dashboard)
    public static double goalZoneX = 115.0; // Goal zone X coordinate in inches
    public static double goalZoneY = 115.0; // Goal zone Y coordinate in inches
    
    // Shooter PIDF Constants
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
    
    // Long range PIDF (>= 6 feet)
    public static double pLong = 0.01;
    public static double iLong = 0.0;
    public static double dLong = 0.0001;
    public static double fLong = 0.00084;
    public static double kVLong = 0.0008;
    public static double kSLong = 0.01;
    public static double I_ZONE_LONG = 250.0;
    
    // Turret servo constants
    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition = 0.275;
    public static double turretRightPosition = 0.745;
    public static double turretMaxAngle = 90.0;
    
    public static double hood1Position = 0.54;        // Short range hood
    public static double hood1PositionLong = 0.45;    // Long range hood (>= 6 feet)
    
    // Linear regression for RPM: RPM = 100 * feet + 1150
    private static final double RPM_SLOPE = 100.0;
    private static final double RPM_INTERCEPT = 1150.0;
    
    private boolean lastA = false;
    private boolean shooting = false;
    private int shootState = 0;
    
    // Store calculated values
    private double calculatedAngle = 0;
    private double calculatedRPM = 0;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(25);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize PedroPathing localization
        poseUpdater = new PoseUpdater(hardwareMap, FConstants.class, LConstants.class);
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

        // Initialize shooter and intake motors
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");

        shootl.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeback.setDirection(DcMotorSimple.Direction.REVERSE);
        intakefront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize servos
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        reargate = hardwareMap.get(Servo.class, "reargate");
        hood1 = hardwareMap.get(Servo.class, "hood 1");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        // Initialize limelight (optional)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
        } catch (Exception e) {
            telemetryA.addData("Limelight", "Not found");
            limelight = null;
        }

        // Set all motors to brake
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Initialize shooter PIDF controller
        shooterPID = new PIDFController(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);
        
        // Initialize timer
        shootTimer = new ElapsedTime();

        // Reset servos to safe positions
        launchgate.setPosition(0.5);
        reargate.setPosition(0.0);
        hood1.setPosition(hood1Position);

        telemetryA.addLine("Auto Shoot Long Range Initialized");
        telemetryA.addLine("=================================");
        telemetryA.addLine("Uses PedroPathing localization");
        telemetryA.addLine("Calculates distance to goal");
        telemetryA.addLine("Auto-aims turret to goal");
        telemetryA.addLine("");
        telemetryA.addLine("Controls:");
        telemetryA.addLine("  A Button = Auto Shoot at Goal");
        telemetryA.addLine("  Right Stick = Drive");
        telemetryA.addLine("  Left Stick X = Rotate");
        telemetryA.addData("Goal Zone", "(%.1f, %.1f)", goalZoneX, goalZoneY);
        telemetryA.update();
    }

    @Override
    public void loop() {
        // Update localization
        poseUpdater.update();
        Pose currentPose = poseUpdater.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();
        
        // Update dashboard
        dashboardPoseTracker.update();
        Drawing.drawRobot(currentPose, "#4CAF50");
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50FF");
        Drawing.sendPacket();

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

        // ==================== CALCULATE DISTANCE AND ANGLE TO GOAL ====================
        double deltaGoalX = goalZoneX - currentX;
        double deltaGoalY = goalZoneY - currentY;
        double distanceToGoalInches = Math.sqrt(deltaGoalX * deltaGoalX + deltaGoalY * deltaGoalY);
        double distanceToGoalFeet = distanceToGoalInches / 12.0;
        
        // Calculate angle to goal (in radians, then convert to degrees)
        double angleToGoalRad = Math.atan2(deltaGoalY, deltaGoalX);
        double angleToGoalDeg = Math.toDegrees(angleToGoalRad);
        
        // Calculate turret angle relative to robot heading
        double turretAngleDeg = angleToGoalDeg - Math.toDegrees(currentHeading);
        
        // Normalize turret angle to -180 to 180
        while (turretAngleDeg > 180) turretAngleDeg -= 360;
        while (turretAngleDeg < -180) turretAngleDeg += 360;
        
        // Clamp to turret limits
        calculatedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDeg));
        
        // Calculate target RPM based on distance
        calculatedRPM = RPM_SLOPE * distanceToGoalFeet + RPM_INTERCEPT;
        calculatedRPM = Math.max(1250.0, Math.min(2500.0, calculatedRPM));  // Increased max for long shots
        
        // Calculate turret servo position
        double turretPos;
        if (calculatedAngle >= 0) {
            turretPos = turretCenterPosition + (calculatedAngle / turretMaxAngle) * (turretRightPosition - turretCenterPosition);
        } else {
            turretPos = turretCenterPosition - (Math.abs(calculatedAngle) / turretMaxAngle) * (turretCenterPosition - turretLeftPosition);
        }
        
        // Set turret position
        turret1.setPosition(turretPos);
        turret2.setPosition(turretPos);
        hood1.setPosition(hood1Position);

        // ==================== AUTO SHOOT CONTROL ====================
        // Detect A button press (edge trigger)
        boolean currentA = gamepad1.a;
        if (currentA && !lastA && !shooting) {
            // Start shooting sequence
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
        
        // ==================== SHOOTER VELOCITY CONTROL ====================
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
        double targetTPS = rpmToTicksPerSec(calculatedRPM);
        
        // Read current velocities
        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double vAvg = 0.5 * (vR + vL);
        
        double shooterPower = 0;
        double pidfOutput = 0;
        double additionalFF = 0;
        
        if (shooting || gamepad1.right_trigger > 0.1) {
            // PIDF control
            pidfOutput = shooterPID.calculate(vAvg, targetTPS);
            
            // Additional feedforward (use current values based on distance)
            double sgn = Math.signum(targetTPS);
            additionalFF = (Math.abs(targetTPS) > 1e-6) ? (currentKS * sgn + currentKV * targetTPS) : 0.0;
            
            // Total power
            shooterPower = pidfOutput + additionalFF;
            shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
            
            shootr.setPower(shooterPower);
            shootl.setPower(shooterPower);
        } else {
            shootr.setPower(0);
            shootl.setPower(0);
            shooterPID.reset();
        }

        // ==================== TELEMETRY ====================
        double avgRPM = ticksPerSecToRPM(vAvg);
        
        telemetryA.addLine("=== AUTO SHOOT LONG RANGE ===");
        telemetryA.addData("Current Pose", String.format("(%.1f, %.1f) @ %.1f°", 
            currentX, currentY, Math.toDegrees(currentHeading)));
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== GOAL TARGETING ===");
        telemetryA.addData("Goal Zone", "(%.1f, %.1f)", goalZoneX, goalZoneY);
        telemetryA.addData("Distance to Goal", "%.1f inches (%.1f feet)", distanceToGoalInches, distanceToGoalFeet);
        telemetryA.addData("Angle to Goal", "%.1f° (field)", angleToGoalDeg);
        telemetryA.addData("Turret Angle", "%.1f° (relative)", calculatedAngle);
        telemetryA.addData("Calculated RPM", "%.0f", calculatedRPM);
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== SHOOTER STATUS ===");
        telemetryA.addData("Distance Range", isLongRange ? "LONG (≥6ft) - p=0.01, hood=0.45" : "SHORT (<6ft) - p=0.002, hood=0.54");
        telemetryA.addData("Status", shootStatus);
        telemetryA.addData("State", shootState);
        telemetryA.addData("Current RPM", "%.0f", avgRPM);
        telemetryA.addData("Error (RPM)", "%.0f", calculatedRPM - avgRPM);
        telemetryA.addData("Shooter Power", "%.3f", shooterPower);
        
        if (shooting || gamepad1.right_trigger > 0.1) {
            telemetryA.addData("PIDF Output", "%.4f", pidfOutput);
            telemetryA.addData("Additional FF", "%.4f", additionalFF);
        }
        
        telemetryA.addData("Right Motor TPS", "%.1f", vR);
        telemetryA.addData("Left Motor TPS", "%.1f", vL);
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== CONTROLS ===");
        telemetryA.addData("A Button", "Auto aim & shoot at goal");
        telemetryA.addData("Right Trigger", "Manual shooter spin");
        telemetryA.addLine("Adjust Goal Zone on Dashboard");
        
        telemetryA.update();
    }

    @Override
    public void stop() {
        // Stop all motors
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        intakefront.setPower(0);
        intakeback.setPower(0);
        shootr.setPower(0);
        shootl.setPower(0);
        
        // Stop limelight if initialized
        if (limelight != null) {
            limelight.stop();
        }
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
}

