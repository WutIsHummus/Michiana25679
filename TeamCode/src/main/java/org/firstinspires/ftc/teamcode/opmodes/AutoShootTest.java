package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Auto Shoot Test")
public class AutoShootTest extends OpMode {

    // Preset shooting parameters (adjustable from dashboard)
    public static double PRESET_DISTANCE_FEET = 3.0;
    public static double PRESET_TURRET_ANGLE = 0.0;
    
    // Shooter PIDF Constants - From VelocityFinder
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;
    public static double p = 0.002;
    public static double i = 0.0;
    public static double d = 0.0001;
    public static double f = 0.00084;
    public static double kV = 0.0008;
    public static double kS = 0.01;
    public static double I_ZONE = 250.0;
    
    // Turret servo constants
    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition = 0.275;
    public static double turretRightPosition = 0.745;
    public static double turretMaxAngle = 90.0;
    
    // Linear regression for RPM: RPM = 100 * feet + 1150
    private static final double RPM_SLOPE = 100.0;
    private static final double RPM_INTERCEPT = 1150.0;
    
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo launchgate, reargate, turret1, turret2, hood1;
    private Limelight3A limelight;
    
    private PIDFController shooterPID;
    private ElapsedTime shootTimer;
    
    private boolean lastA = false;
    private boolean shooting = false;
    private int shootState = 0;
    
    public static double hood1Position = 0.54;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
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
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        hood1 = hardwareMap.get(Servo.class, "hood 1");

        // Initialize limelight (optional, can be null if not used)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
        } catch (Exception e) {
            telemetry.addData("Limelight", "Not found - continuing without it");
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

        telemetry.addLine("Auto Shoot Test Initialized");
        telemetry.addLine("=========================");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A Button = Auto Shoot at Preset");
        telemetry.addLine("  Right Stick = Drive");
        telemetry.addLine("  Left Stick X = Rotate");
        telemetry.addLine("");
        telemetry.addData("Preset Distance", "%.1f feet", PRESET_DISTANCE_FEET);
        telemetry.addData("Preset Turret Angle", "%.1f degrees", PRESET_TURRET_ANGLE);
        telemetry.addLine("");
        telemetry.addLine("Adjust presets on FTC Dashboard");
        telemetry.update();
    }

    @Override
    public void loop() {
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

        // ==================== AUTO SHOOT CONTROL ====================
        // Calculate target RPM based on preset distance
        double targetRPM = RPM_SLOPE * PRESET_DISTANCE_FEET + RPM_INTERCEPT;
        targetRPM = Math.max(1250.0, Math.min(1750.0, targetRPM));
        
        // Calculate turret servo position
        double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, PRESET_TURRET_ANGLE));
        double turretPos;
        if (clampedAngle >= 0) {
            turretPos = turretCenterPosition + (clampedAngle / turretMaxAngle) * (turretRightPosition - turretCenterPosition);
        } else {
            turretPos = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * (turretCenterPosition - turretLeftPosition);
        }
        
        // Set turret position
        turret1.setPosition(turretPos);
        turret2.setPosition(turretPos);
        hood1.setPosition(hood1Position);
        
        // Detect A button press (edge trigger)
        boolean currentA = gamepad1.a;
        if (currentA && !lastA && !shooting) {
            // Start shooting sequence
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }
        lastA = currentA;

        // Run shooting state machine
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
                    
                case 1: // Run intakes
                    shootStatus = "Running intakes...";
                    intakefront.setPower(-1.0);
                    intakeback.setPower(-1.0);
                    if (shootTimer.seconds() > 0.3) {
                        shootState = 2;
                        shootTimer.reset();
                    }
                    break;
                    
                case 2: // Fire launch gate
                    shootStatus = "Firing!";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2) {
                        shootState = 3;
                        shootTimer.reset();
                    }
                    break;
                    
                case 3: // Reset launch gate
                    shootStatus = "Resetting...";
                    launchgate.setPosition(0.5);
                    intakefront.setPower(0);
                    intakeback.setPower(0);
                    if (shootTimer.seconds() > 0.2) {
                        shooting = false;
                        shootState = 0;
                        shootStatus = "Complete!";
                    }
                    break;
            }
        }
        
        // ==================== SHOOTER VELOCITY CONTROL ====================
        // Update PIDF coefficients
        shooterPID.setPIDF(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);
        
        // Convert target RPM to ticks per second
        double targetTPS = rpmToTicksPerSec(targetRPM);
        
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
            
            // Additional feedforward
            double sgn = Math.signum(targetTPS);
            additionalFF = (Math.abs(targetTPS) > 1e-6) ? (kS * sgn + kV * targetTPS) : 0.0;
            
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
        
        telemetry.addLine("=== AUTO SHOOT TEST ===");
        telemetry.addData("Drive", String.format("FL:%.2f FR:%.2f BL:%.2f BR:%.2f", 
            flPower, frPower, blPower, brPower));
        telemetry.addData("", "");
        
        telemetry.addLine("=== PRESET PARAMETERS ===");
        telemetry.addData("Distance", "%.1f feet", PRESET_DISTANCE_FEET);
        telemetry.addData("Turret Angle", "%.1f degrees", PRESET_TURRET_ANGLE);
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Turret Servo Pos", "%.3f", turretPos);
        telemetry.addData("", "");
        
        telemetry.addLine("=== SHOOTER STATUS ===");
        telemetry.addData("Status", shootStatus);
        telemetry.addData("State", shootState);
        telemetry.addData("Current RPM", "%.0f", avgRPM);
        telemetry.addData("Error (RPM)", "%.0f", targetRPM - avgRPM);
        telemetry.addData("Shooter Power", "%.3f", shooterPower);
        
        if (shooting || gamepad1.right_trigger > 0.1) {
            telemetry.addData("PIDF Output", "%.4f", pidfOutput);
            telemetry.addData("Additional FF", "%.4f", additionalFF);
        }
        
        telemetry.addData("Right Motor TPS", "%.1f", vR);
        telemetry.addData("Left Motor TPS", "%.1f", vL);
        telemetry.addData("", "");
        
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A Button", "Auto shoot sequence");
        telemetry.addData("Right Trigger", "Manual shooter spin");
        telemetry.addLine("Adjust Distance/Angle on Dashboard");
        
        telemetry.update();
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

