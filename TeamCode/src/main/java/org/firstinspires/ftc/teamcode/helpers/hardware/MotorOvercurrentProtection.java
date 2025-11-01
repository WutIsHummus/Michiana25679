package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;
import java.util.Map;

public class MotorOvercurrentProtection {
    
    private final Map<String, MotorMonitor> monitoredMotors;
    private final Telemetry telemetry;
    private boolean enableProtection = true;
    private boolean enableTelemetry = true;
    
    private static final double DEFAULT_THRESHOLD_AMPS = 4.5;
    
    public MotorOvercurrentProtection(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.monitoredMotors = new HashMap<>();
    }
    
    public void addMotor(String name, DcMotorEx motor) {
        addMotor(name, motor, DEFAULT_THRESHOLD_AMPS);
    }
    
    public void addMotor(String name, DcMotorEx motor, double thresholdAmps) {
        monitoredMotors.put(name, new MotorMonitor(motor, thresholdAmps));
    }
    
    public void removeMotor(String name) {
        monitoredMotors.remove(name);
    }
    
    public void setThreshold(String name, double thresholdAmps) {
        MotorMonitor monitor = monitoredMotors.get(name);
        if (monitor != null) {
            monitor.thresholdAmps = thresholdAmps;
        }
    }
    
    public void setProtectionEnabled(boolean enabled) {
        this.enableProtection = enabled;
    }
    
    public void setTelemetryEnabled(boolean enabled) {
        this.enableTelemetry = enabled;
    }
    
    public boolean checkAndProtect() {
        boolean anyOvercurrent = false;
        
        for (Map.Entry<String, MotorMonitor> entry : monitoredMotors.entrySet()) {
            String motorName = entry.getKey();
            MotorMonitor monitor = entry.getValue();
            
            double currentAmps = monitor.motor.getCurrent(CurrentUnit.AMPS);
            boolean isOverCurrent = currentAmps > monitor.thresholdAmps;
            
            if (isOverCurrent) {
                monitor.overcurrentCount++;
                anyOvercurrent = true;
                
                if (enableProtection) {
                    monitor.motor.setPower(0.0);
                    monitor.lastStoppedTime = System.currentTimeMillis();
                }
                
                if (enableTelemetry) {
                    telemetry.addLine("⚠️ OVERCURRENT: " + motorName);
                    telemetry.addData(motorName + " Current", String.format("%.2f A", currentAmps));
                    telemetry.addData(motorName + " Threshold", String.format("%.2f A", monitor.thresholdAmps));
                }
            } else if (enableTelemetry) {
                telemetry.addData(motorName + " Current", String.format("%.2f A / %.2f A", 
                    currentAmps, monitor.thresholdAmps));
            }
        }
        
        return anyOvercurrent;
    }
    
    public boolean isMotorOverCurrent(String name) {
        MotorMonitor monitor = monitoredMotors.get(name);
        if (monitor == null) return false;
        
        double currentAmps = monitor.motor.getCurrent(CurrentUnit.AMPS);
        return currentAmps > monitor.thresholdAmps;
    }
    
    public double getMotorCurrent(String name) {
        MotorMonitor monitor = monitoredMotors.get(name);
        if (monitor == null) return -1;
        
        return monitor.motor.getCurrent(CurrentUnit.AMPS);
    }
    
    public int getOvercurrentCount(String name) {
        MotorMonitor monitor = monitoredMotors.get(name);
        if (monitor == null) return 0;
        
        return monitor.overcurrentCount;
    }
    
    public void resetOvercurrentCount(String name) {
        MotorMonitor monitor = monitoredMotors.get(name);
        if (monitor != null) {
            monitor.overcurrentCount = 0;
        }
    }
    
    public void resetAllCounters() {
        for (MotorMonitor monitor : monitoredMotors.values()) {
            monitor.overcurrentCount = 0;
        }
    }
    
    public void printDetailedStatus() {
        if (!enableTelemetry) return;
        
        telemetry.addLine("=== Motor Overcurrent Protection ===");
        telemetry.addData("Protection", enableProtection ? "ENABLED" : "MONITORING ONLY");
        telemetry.addData("Motors Monitored", monitoredMotors.size());
        telemetry.addLine();
        
        for (Map.Entry<String, MotorMonitor> entry : monitoredMotors.entrySet()) {
            String motorName = entry.getKey();
            MotorMonitor monitor = entry.getValue();
            
            double currentAmps = monitor.motor.getCurrent(CurrentUnit.AMPS);
            double power = monitor.motor.getPower();
            boolean isOverCurrent = currentAmps > monitor.thresholdAmps;
            
            telemetry.addData(motorName + " Status", isOverCurrent ? "⚠️ OVER" : "✓ OK");
            telemetry.addData(motorName + " Current", String.format("%.2f / %.2f A", 
                currentAmps, monitor.thresholdAmps));
            telemetry.addData(motorName + " Power", String.format("%.2f", power));
            telemetry.addData(motorName + " Events", monitor.overcurrentCount);
        }
    }
    
    private static class MotorMonitor {
        DcMotorEx motor;
        double thresholdAmps;
        int overcurrentCount;
        long lastStoppedTime;
        
        MotorMonitor(DcMotorEx motor, double thresholdAmps) {
            this.motor = motor;
            this.thresholdAmps = thresholdAmps;
            this.overcurrentCount = 0;
            this.lastStoppedTime = 0;
        }
    }
    
    public static MotorOvercurrentProtection createForRobot(
            Telemetry telemetry,
            DcMotorEx liftMotor,
            DcMotorEx liftMotor2,
            DcMotorEx extendoMotor,
            DcMotorEx spinMotor) {
        
        MotorOvercurrentProtection protection = new MotorOvercurrentProtection(telemetry);
        
        if (liftMotor != null) {
            protection.addMotor("Lift R", liftMotor, 5.0);
        }
        if (liftMotor2 != null) {
            protection.addMotor("Lift L", liftMotor2, 5.0);
        }
        if (extendoMotor != null) {
            protection.addMotor("Extendo", extendoMotor, 4.5);
        }
        if (spinMotor != null) {
            protection.addMotor("Spin", spinMotor, 3.5);
        }
        
        return protection;
    }
}
