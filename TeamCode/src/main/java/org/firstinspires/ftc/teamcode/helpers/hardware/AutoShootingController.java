package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Auto Shooting Controller - Extracted from FullTesting
 * Handles velocity-based shooting with distance-based PID tuning
 */
@Config
public class AutoShootingController {
    
    // Hardware
    private final DcMotorEx shootr, shootl;
    private final Servo hood1;
    private final VoltageSensor voltageSensor;
    private final PIDFController shooterPID;
    
    // Constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;
    private static final double NOMINAL_VOLTAGE = 12.0;
    
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
    
    // RPM calculation constants
    private static final double RPM_SLOPE = 100.0;
    private static final double RPM_INTERCEPT = 1150.0;
    public static double FAR_SHOOTING_RPM_MAX = 1950.0;
    public static double RPM_TOLERANCE = 50.0;
    
    // Goal zone coordinates
    public static double goalZoneX = 116.0;
    public static double goalZoneY = 116.0;
    
    public AutoShootingController(HardwareMap hardwareMap) {
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");
        hood1 = hardwareMap.get(Servo.class, "hood 1");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        
        shooterPID = new PIDFController(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);
    }
    
    /**
     * Update shooter velocity control (call this in loop())
     * EXACTLY like FullTesting lines 395-482
     * 
     * @param shooterOn Whether shooter should be running
     * @param currentX Robot X position in inches
     * @param currentY Robot Y position in inches
     * @return ShooterStatus containing telemetry info
     */
    public ShooterStatus update(boolean shooterOn, double currentX, double currentY) {
        // Calculate distance to goal zone for RPM calculation
        double deltaGoalX = goalZoneX - currentX;
        double deltaGoalY = goalZoneY - currentY;
        double distanceToGoalInches = Math.sqrt(deltaGoalX * deltaGoalX + deltaGoalY * deltaGoalY);
        double distanceToGoalFeet = distanceToGoalInches / 12.0;
        
        // Calculate target RPM using linear regression
        double calculatedTargetRPM;
        if (distanceToGoalFeet >= 9.0) {
            calculatedTargetRPM = FAR_SHOOTING_RPM_MAX;
        } else {
            calculatedTargetRPM = RPM_SLOPE * distanceToGoalFeet + RPM_INTERCEPT;
            calculatedTargetRPM = Math.max(1250.0, Math.min(FAR_SHOOTING_RPM_MAX, calculatedTargetRPM));
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
        double compensatedPower = 0;
        
        if (shooterOn) {
            // PIDF control (F term is built-in and multiplied by setpoint)
            pidfOutput = shooterPID.calculate(vAvg, targetTPS);
            
            // Additional feedforward using kV and kS
            double sgn = Math.signum(targetTPS);
            additionalFF = (Math.abs(targetTPS) > 1e-6) ? (currentKS * sgn + currentKV * targetTPS) : 0.0;
            
            // Total power
            shooterPower = pidfOutput + additionalFF;
            
            // Safety: prevent overshoot
            if (avgVelocityRPM >= calculatedTargetRPM && shooterPower > 0) {
                shooterPower = Math.min(shooterPower, 0.5);
            }
            
            // Clamp
            shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
            
            // Voltage compensation (always on): power * (12V / currentVoltage)
            double voltage = voltageSensor.getVoltage();
            compensatedPower = shooterPower * (NOMINAL_VOLTAGE / voltage);
            compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));
            
            shootr.setPower(compensatedPower);
            shootl.setPower(compensatedPower);
        } else {
            shootr.setPower(0);
            shootl.setPower(0);
            shooterPID.reset();
        }
        
        // Return status for telemetry
        boolean atTargetSpeed = Math.abs(avgVelocityRPM - calculatedTargetRPM) < RPM_TOLERANCE;
        return new ShooterStatus(
            calculatedTargetRPM, avgVelocityRPM, shootrVelocityRPM, shootlVelocityRPM,
            distanceToGoalInches, distanceToGoalFeet, isLongRange,
            shooterPower, compensatedPower, pidfOutput, additionalFF,
            atTargetSpeed, currentP, currentI, currentD, currentF
        );
    }
    
    /**
     * Stop the shooter
     */
    public void stop() {
        shootr.setPower(0);
        shootl.setPower(0);
        shooterPID.reset();
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
    
    /**
     * Status class for telemetry
     */
    public static class ShooterStatus {
        public final double targetRPM;
        public final double avgRPM;
        public final double rightRPM;
        public final double leftRPM;
        public final double distanceInches;
        public final double distanceFeet;
        public final boolean isLongRange;
        public final double power;
        public final double compensatedPower;
        public final double pidfOutput;
        public final double additionalFF;
        public final boolean atTargetSpeed;
        public final double activeP;
        public final double activeI;
        public final double activeD;
        public final double activeF;
        
        public ShooterStatus(double targetRPM, double avgRPM, double rightRPM, double leftRPM,
                           double distanceInches, double distanceFeet, boolean isLongRange,
                           double power, double compensatedPower, double pidfOutput, double additionalFF,
                           boolean atTargetSpeed, double activeP, double activeI, double activeD, double activeF) {
            this.targetRPM = targetRPM;
            this.avgRPM = avgRPM;
            this.rightRPM = rightRPM;
            this.leftRPM = leftRPM;
            this.distanceInches = distanceInches;
            this.distanceFeet = distanceFeet;
            this.isLongRange = isLongRange;
            this.power = power;
            this.compensatedPower = compensatedPower;
            this.pidfOutput = pidfOutput;
            this.additionalFF = additionalFF;
            this.atTargetSpeed = atTargetSpeed;
            this.activeP = activeP;
            this.activeI = activeI;
            this.activeD = activeD;
            this.activeF = activeF;
        }
        
        public double getErrorRPM() {
            return targetRPM - avgRPM;
        }
    }
}

