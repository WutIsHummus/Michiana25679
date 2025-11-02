package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@TeleOp(name = "Shooter PID")
public class ShooterPID extends OpMode {

    public static double p = 0.0000, i = 0.0, d = 0.0000;
    public static double kS = 0.00;       // static feedforward 
    public static double kV = 0.0000;     // velocity feedforward (power per tick/s)
    public static double targetRPM = 2500;
    public static double I_ZONE = 250.0;  // integrator clamp range 

    public static int TICKS_PER_REV = 28;
    public static double GEAR_RATIO = 1.0;

    // Single PID
    private PIDController pid;

    // Motors
    private CachingDcMotorEx motorR, motorL;
    private DcMotorEx motorREx, motorLEx;

    private static double rpmToTicksPerSec(double rpm) {
        double motorRPM = rpm * GEAR_RATIO;
        return (motorRPM / 60.0) * TICKS_PER_REV;
    }

    private static double ticksPerSecToRPM(double tps) {
        double motorRPM = (tps / TICKS_PER_REV) * 60.0;
        return motorRPM / GEAR_RATIO;
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx rawR = hardwareMap.get(DcMotorEx.class, "shootr"); 
        DcMotorEx rawL = hardwareMap.get(DcMotorEx.class, "shootl");

        motorR = new CachingDcMotorEx(rawR);
        motorL = new CachingDcMotorEx(rawL);

        motorREx = rawR;
        motorLEx = rawL;

        motorL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(p, i, d);
        pid.setIntegratorRange(-I_ZONE, I_ZONE);

        telemetry.update();
    }

    @Override
    public void loop() {
        pid.setPID(p, i, d);

        // Target (motor shaft units)
        double targetTPS = rpmToTicksPerSec(targetRPM);

        // Measure each motor velocity (ticks/sec)
        double vR = motorREx.getVelocity();
        double vL = motorLEx.getVelocity();

        // Average velocity
        double vAvg = 0.5 * (vR + vL);

        // PID on average
        double u = pid.calculate(vAvg, targetTPS);

        // Feedforward
        double sgn = Math.signum(targetTPS);
        double ff = (Math.abs(targetTPS) > 1e-6) ? (kS * sgn + kV * targetTPS) : 0.0;

        // Same power to both wheels
        double power = u + ff;

        // Clamp
        power = Math.max(-1.0, Math.min(1.0, power));

        // Apply
        motorR.setPower(power);
        motorL.setPower(power);

        // Telemetry (no current/overcurrent)
        telemetry.addData("Current Tick", motor.getCurrentPosition());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Target TPS", String.format("%.1f", targetTPS));
        telemetry.addLine();

        telemetry.addLine("--- Right ---");
        telemetry.addData("vR (TPS)", String.format("%.1f", vR));
        telemetry.addData("vR (RPM)", String.format("%.1f", ticksPerSecToRPM(vR)));

        telemetry.addLine("--- Left ---");
        telemetry.addData("vL (TPS)", String.format("%.1f", vL));
        telemetry.addData("vL (RPM)", String.format("%.1f", ticksPerSecToRPM(vL)));

        telemetry.addLine("--- Controller ---");
        telemetry.addData("vAvg (TPS)", String.format("%.1f", vAvg));
        telemetry.addData("vAvg (RPM)", String.format("%.1f", ticksPerSecToRPM(vAvg)));
        telemetry.addData("PID_u", String.format("%.3f", u));
        telemetry.addData("FF", String.format("%.3f", ff));
        telemetry.addData("Power", String.format("%.3f", power));

        telemetry.addLine();
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("kS", kS);
        telemetry.addData("kV", kV);
        telemetry.addData("I_ZONE (|err| TPS)", I_ZONE);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (motorR != null) motorR.setPower(0);
        if (motorL != null) motorL.setPower(0);
    }
}
