package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@TeleOp(name = "Shooter PID")
public class ShooterPID extends OpMode {

    public static double p = 0.0000, i = 0.0, d = 0.0000;
    public static double f = 0.0;         // simple feedforward (F*setpoint)
    public static double kS = 0.00;       // static feedforward 
    public static double kV = 0.0000;     // velocity feedforward (power per tick/s)
    public static double targetRPM = 2500;
    public static double I_ZONE = 250.0;  // integrator clamp range 

    public static int TICKS_PER_REV = 28;
    public static double GEAR_RATIO = 1.0;

    // Single PIDF Controller
    private PIDFController pidf;

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

        pidf = new PIDFController(p, i, d, f);
        pidf.setIntegrationBounds(-I_ZONE, I_ZONE);

        telemetry.update();
    }

    @Override
    public void loop() {
        pidf.setPIDF(p, i, d, f);

        // Target (motor shaft units)
        double targetTPS = rpmToTicksPerSec(targetRPM);

        // Measure each motor velocity (ticks/sec)
        double vR = motorREx.getVelocity();
        double vL = motorLEx.getVelocity();

        // Average velocity
        double vAvg = 0.5 * (vR + vL);

        // PIDF calculation (F term is built into FTCLib's PIDFController)
        double pidfOutput = pidf.calculate(vAvg, targetTPS);

        // Additional feedforward terms (kS for static friction, kV for velocity)
        double sgn = Math.signum(targetTPS);
        double ff = (Math.abs(targetTPS) > 1e-6) ? (kS * sgn + kV * targetTPS) : 0.0;

        // Total power = PIDF + additional feedforward
        double power = pidfOutput + ff;

        // Clamp
        power = Math.max(-1.0, Math.min(1.0, power));

        // Apply
        motorR.setPower(power);
        motorL.setPower(power);

        // Telemetry (no current/overcurrent)
        telemetry.addData("Current Tick", motorR.getCurrentPosition());
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
        telemetry.addData("PIDF Output", String.format("%.3f", pidfOutput));
        telemetry.addData("Additional FF (kS+kV)", String.format("%.3f", ff));
        telemetry.addData("Total Power", String.format("%.3f", power));

        telemetry.addLine();
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("F", f);
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