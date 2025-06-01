package org.firstinspires.ftc.teamcode.helpers.hardware;

public class SquIDController {
    double p, i, d;
    public SquIDController(double p, double i,double d) {
        this.p=p;
        this.i=i;
        this.d=d;
    }
    public void setPID(double p) {
        this.p = p;
    }
    public double calculate(double setpoint, double current) {
        return Math.sqrt(Math.abs((current-setpoint)*p))*Math.signum(current-setpoint);
    }
}
