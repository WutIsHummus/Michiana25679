package org.firstinspires.ftc.teamcode.opmodes;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * Shooting While Moving Example
 * 
 * Demonstrates shooting at a target while the robot is moving.
 * Features:
 * - Velocity-based RPM compensation
 * - Predictive aiming (leads target based on robot velocity)
 * - REV Hub native velocity PIDF control
 * - Automatic distance-based RPM calculation
 * 
 * Controls:
 * - Right Stick: Drive robot
 * - A: Toggle shooter on/off
 * - B: Shoot (when shooter ready)
 * - D-pad UP/DOWN: Adjust base target RPM
 * - Right Trigger: Low-power mode
 */
@Config
@TeleOp(name = "Shooting While Moving", group = "Examples")
public class ShootingWhileMovingExample extends OpMode {

    /* ================= SHOOTER CONFIGURATION ================= */
    
    // Motor encoder configuration
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 20.0 / 23.0;  // Adjust to match your gear ratio
    
    // Base RPM settings
    public static double BASE_TARGET_RPM = 1100.0;
    public static double LOW_POWER_RPM = 800.0;
    public static double RPM_STEP = 50.0;
    
    // Velocity PIDF coefficients (from tuner)
    public static double kP = 10.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 11.7;
    
    // Velocity compensation settings
    public static boolean USE_VELOCITY_COMPENSATION = true;
    public static double VELOCITY_COMPENSATION_FACTOR = 15.0;  // RPM per inch/sec of forward velocity
    
    // RPM ramp settings
    public static boolean USE_RAMP = true;
    public static double MAX_RPM_CHANGE_PER_SEC = 3000.0;
    
    // Ready check settings
    public static double READY_RPM_ERROR = 40.0;
    public static double READY_TIME_SEC = 0.25;
    
    // Voltage compensation
    public static boolean USE_VOLTAGE_COMP = true;
    public static double NOMINAL_VOLTAGE = 12.0;
    
    /* ================= TURRET CONFIGURATION ================= */
    
    // Target coordinates (adjust for your field/color)
    public static double TARGET_X = 118.0;  // Goal center X
    public static double TARGET_Y = 125.0;  // Goal center Y
    
    // Turret servo positions
    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition = 0.15;
    public static double turretRightPosition = 0.85;
    public static double turretMaxAngle = 140.0;
    
    // Predictive aiming
    public static boolean USE_PREDICTIVE_AIM = true;
    public static double PROJECTILE_TRAVEL_TIME = 0.3;  // Estimated seconds for projectile to reach target
    
    /* ================= HARDWARE ================= */
    
    private Follower follower;
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx shootR, shootL;
    private Servo turret1, turret2;
    private VoltageSensor battery;
    
    /* ================= STATE ================= */
    
    private boolean shooterEnabled = false;
    private boolean shooting = false;
    private double commandedRPM = 0;
    private double stableTimer = 0;
    private long lastLoopTime;
    private Pose lastPose;
    private double lastPoseTime;
    
    // Gamepad state
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    
    private ElapsedTime loopClock = new ElapsedTime();
    
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize Follower for localization
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
        
        // Initialize shooter motors
        shootR = hardwareMap.get(DcMotorEx.class, "shootr");
        shootL = hardwareMap.get(DcMotorEx.class, "shootl");
        shootL.setDirection(DcMotorSimple.Direction.REVERSE);
        
        for (DcMotorEx m : new DcMotorEx[]{shootR, shootL}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
        // Initialize turret servos (if you have them)
        try {
            turret1 = hardwareMap.get(Servo.class, "turret1");
            turret2 = hardwareMap.get(Servo.class, "turret2");
        } catch (Exception e) {
            turret1 = null;
            turret2 = null;
            telemetry.addLine("Warning: Turret servos not found");
        }
        
        battery = hardwareMap.voltageSensor.iterator().next();
        
        lastLoopTime = System.nanoTime();
        lastPoseTime = loopClock.seconds();
        lastPose = new Pose(0, 0, 0);
        
        telemetry.addLine("Shooting While Moving Example Initialized");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        double dt = loopDt();
        
        // Update localization
        follower.update();
        Pose currentPose = follower.getPose();
        
        // Calculate robot velocity
        double vxField = 0.0, vyField = 0.0;
        double nowSec = loopClock.seconds();
        if (lastPose != null) {
            double poseDt = nowSec - lastPoseTime;
            if (poseDt > 1e-3) {
                vxField = (currentPose.getX() - lastPose.getX()) / poseDt;
                vyField = (currentPose.getY() - lastPose.getY()) / poseDt;
            }
        }
        lastPose = currentPose;
        lastPoseTime = nowSec;
        
        double robotSpeed = Math.hypot(vxField, vyField);
        double robotHeading = currentPose.getHeading();
        
        // Calculate velocity component in robot's forward direction
        double forwardVelocity = vxField * Math.cos(robotHeading) + vyField * Math.sin(robotHeading);
        
        // Update controls
        updateControls();
        
        // Drive robot
        driveRobot();
        
        // Update shooter
        updateShooter(dt, forwardVelocity);
        
        // Update turret aiming (with prediction if enabled)
        updateTurretAiming(currentPose, vxField, vyField);
        
        // Update telemetry
        updateTelemetry(currentPose, vxField, vyField, robotSpeed);

        sleep(20);
    }
    
    private void updateControls() {
        boolean aPressed = gamepad1.a;
        boolean bPressed = gamepad1.b;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        
        // Toggle shooter
        if (aPressed && !lastA) {
            shooterEnabled = !shooterEnabled;
        }
        
        // Shoot button
        if (bPressed && !lastB && shooterEnabled) {
            shooting = true;
            // Add your shooting action here (launch gate, etc.)
        }
        lastB = bPressed;
        
        // Adjust target RPM
        if (dpadUp && !lastDpadUp) {
            BASE_TARGET_RPM += RPM_STEP;
        }
        if (dpadDown && !lastDpadDown) {
            BASE_TARGET_RPM = Math.max(0, BASE_TARGET_RPM - RPM_STEP);
        }
        
        lastA = aPressed;
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
    }
    
    private void driveRobot() {
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x * 1.1;
        double rx = gamepad1.left_stick_x;
        
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        
        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;
        
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
    
    private void updateShooter(double dt, double forwardVelocity) {
        // Calculate distance to target
        Pose currentPose = follower.getPose();
        double dx = TARGET_X - currentPose.getX();
        double dy = TARGET_Y - currentPose.getY();
        double distance = Math.hypot(dx, dy);
        
        // Base RPM from distance (you can use your regression here)
        double targetRPM = BASE_TARGET_RPM;
        
        // Velocity compensation: increase RPM when moving forward
        if (USE_VELOCITY_COMPENSATION && forwardVelocity > 0) {
            targetRPM += forwardVelocity * VELOCITY_COMPENSATION_FACTOR;
        }
        
        // Low power mode
        if (gamepad1.right_trigger > 0.1) {
            targetRPM = LOW_POWER_RPM;
        }
        
        // Apply voltage compensation to kF
        double voltage = battery.getVoltage();
        double fScale = USE_VOLTAGE_COMP ? (NOMINAL_VOLTAGE / voltage) : 1.0;
        
        shootR.setVelocityPIDFCoefficients(kP, kI, kD, kF * fScale);
        shootL.setVelocityPIDFCoefficients(kP, kI, kD, kF * fScale);
        
        if (!shooterEnabled) {
            shootR.setVelocity(0);
            shootL.setVelocity(0);
            commandedRPM = 0;
            stableTimer = 0;
        } else {
            // Ramp RPM smoothly
            commandedRPM = USE_RAMP
                    ? ramp(commandedRPM, targetRPM, dt)
                    : targetRPM;
            
            double targetTPS = rpmToTicks(commandedRPM);
            shootR.setVelocity(targetTPS);
            shootL.setVelocity(targetTPS);
        }
        
        // Ready check
        double rpmR = ticksToRPM(shootR.getVelocity());
        double rpmL = ticksToRPM(shootL.getVelocity());
        double avgRPM = (rpmR + rpmL) / 2.0;
        
        if (Math.abs(avgRPM - targetRPM) < READY_RPM_ERROR) {
            stableTimer += dt;
        } else {
            stableTimer = 0;
        }
    }
    
    private void updateTurretAiming(Pose currentPose, double vxField, double vyField) {
        if (turret1 == null || turret2 == null) return;
        
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();
        
        // Calculate target position (with prediction if enabled)
        double aimX = currentX;
        double aimY = currentY;
        
        if (USE_PREDICTIVE_AIM) {
            // Predict where we'll be when projectile reaches target
            aimX = currentX + vxField * PROJECTILE_TRAVEL_TIME;
            aimY = currentY + vyField * PROJECTILE_TRAVEL_TIME;
        }
        
        // Calculate vector to target from aim position
        double dx = TARGET_X - aimX;
        double dy = TARGET_Y - aimY;
        
        // Calculate angle to target
        double angleToTargetField = Math.atan2(dy, dx);
        double turretAngle = normalizeAngle(angleToTargetField - currentHeading);
        
        // Convert to servo position
        double turretAngleDeg = Math.toDegrees(turretAngle);
        double clampedDeg = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDeg));
        
        double servoPosition;
        if (clampedDeg >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedDeg / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedDeg) / turretMaxAngle) * servoRange;
        }
        
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
        
        if (turret1 != null) turret1.setPosition(servoPosition);
        if (turret2 != null) turret2.setPosition(servoPosition);
    }
    
    private void updateTelemetry(Pose pose, double vx, double vy, double speed) {
        double rpmR = ticksToRPM(shootR.getVelocity());
        double rpmL = ticksToRPM(shootL.getVelocity());
        double avgRPM = (rpmR + rpmL) / 2.0;
        boolean ready = stableTimer > READY_TIME_SEC;
        
        telemetry.addLine("=== ROBOT STATE ===");
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Velocity", "(%.1f, %.1f) in/s", vx, vy);
        telemetry.addData("Speed", "%.1f in/s", speed);
        
        telemetry.addLine();
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Enabled", shooterEnabled ? "YES" : "NO");
        telemetry.addData("Target RPM", "%.1f", BASE_TARGET_RPM + (vx * VELOCITY_COMPENSATION_FACTOR));
        telemetry.addData("Commanded RPM", "%.1f", commandedRPM);
        telemetry.addData("Avg RPM", "%.1f", avgRPM);
        telemetry.addData("Ready", ready ? "YES" : "NO");
        telemetry.addData("Velocity Comp", USE_VELOCITY_COMPENSATION && vx > 0 ? "ON" : "OFF");
        
        telemetry.addLine();
        telemetry.addLine("=== TURRET ===");
        double dx = TARGET_X - pose.getX();
        double dy = TARGET_Y - pose.getY();
        double dist = Math.hypot(dx, dy);
        telemetry.addData("Distance to Target", "%.1f in", dist);
        telemetry.addData("Predictive Aim", USE_PREDICTIVE_AIM ? "ON" : "OFF");
        
        telemetry.update();
    }
    
    /* ================= HELPER METHODS ================= */
    
    private double loopDt() {
        long now = System.nanoTime();
        double dt = (now - lastLoopTime) / 1e9;
        lastLoopTime = now;
        return Math.max(dt, 1e-3);
    }
    
    private double ramp(double current, double target, double dt) {
        double maxDelta = MAX_RPM_CHANGE_PER_SEC * dt;
        double delta = target - current;
        if (Math.abs(delta) <= maxDelta) return target;
        return current + Math.signum(delta) * maxDelta;
    }
    
    private double rpmToTicks(double rpm) {
        double motorRPM = rpm * GEAR_RATIO;
        return (motorRPM / 60.0) * TICKS_PER_REV;
    }
    
    private double ticksToRPM(double tps) {
        double motorRPM = (tps / TICKS_PER_REV) * 60.0;
        return motorRPM / GEAR_RATIO;
    }
    
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
