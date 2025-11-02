package org.firstinspires.ftc.teamcode.helpers.hardware;

public class VelocityPIDFController {
    
    public double kS = 0.05;
    public double kV = 0.00018;
    public double kA = 0.0;
    
    public double kP = 0.0005;
    public double kI = 0.0;
    public double kD = 0.0001;
    
    private double integral = 0;
    private double lastError = 0;
    private double lastVel = 0;
    private long lastT = System.nanoTime();
    
    private double integralMin = -2000;
    private double integralMax = 2000;
    
    public VelocityPIDFController(double kS, double kV, double kA, double kP, double kI, double kD) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
    public VelocityPIDFController() {
    }
    
    public void reset() {
        integral = 0;
        lastError = 0;
        lastVel = 0;
        lastT = System.nanoTime();
    }
    
    public void setIntegralBounds(double min, double max) {
        this.integralMin = min;
        this.integralMax = max;
    }
    
    public double update(double targetTPS, double measuredTPS) {
        long now = System.nanoTime();
        double dt = Math.max((now - lastT) / 1e9, 1e-3);
        lastT = now;
        
        double sign = Math.signum(targetTPS);
        double accel = (measuredTPS - lastVel) / dt;
        double uFF = kS * sign + kV * targetTPS + kA * accel;
        
        double error = targetTPS - measuredTPS;
        integral += error * dt;
        integral = clamp(integral, integralMin, integralMax);
        double deriv = (error - lastError) / dt;
        lastError = error;
        lastVel = measuredTPS;
        
        double uPID = kP * error + kI * integral + kD * deriv;
        return clamp(uFF + uPID, -1.0, 1.0);
    }
    
    public double getLastError() {
        return lastError;
    }
    
    public double getIntegral() {
        return integral;
    }
    
    public double getDerivative() {
        return (lastError - 0) / Math.max((System.nanoTime() - lastT) / 1e9, 1e-3);
    }
    
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

