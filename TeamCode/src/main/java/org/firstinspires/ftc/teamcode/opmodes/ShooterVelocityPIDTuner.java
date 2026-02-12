package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp(name = "Shooter Velocity PIDF (6000RPM 23:20)", group = "Tuning")
@Disabled
public class ShooterVelocityPIDTuner extends LinearOpMode {

    /* ================= ENCODER / GEARING ================= */

    // goBILDA Yellow Jacket encoder (motor shaft)
    public static double TICKS_PER_REV = 28.0;

    // 23T motor -> 20T flywheel (speed-up)
    // motorRPM = flywheelRPM * (20/23)
    public static double GEAR_RATIO = 20.0 / 23.0;

    /* ================= TARGET SPEEDS ================= */

    public static double TARGET_RPM = 1100;
    public static double LOW_POWER_RPM = 800;
    public static double RPM_STEP = 50;

    /* ================= VELOCITY PIDF (REV HUB) ================= */

    // Starting values calculated for 6000 RPM goBILDA
    public static double kP = 10.0;
    public static double kI = 0.0;     // DO NOT use I for shooters
    public static double kD = 0.0;
    public static double kF = 11.7;    // 32767 / ~2800 TPS

    /* ================= VOLTAGE COMP ================= */

    public static boolean USE_VOLTAGE_COMP = true;
    public static double NOMINAL_VOLTAGE = 12.0;

    /* ================= VELOCITY RAMP ================= */

    public static boolean USE_RAMP = true;
    public static double MAX_RPM_CHANGE_PER_SEC = 3000;

    /* ================= READY CHECK ================= */

    public static double READY_RPM_ERROR = 40;   // ± RPM window
    public static double READY_TIME_SEC = 0.25; // must be stable this long

    /* ================= HARDWARE ================= */

    private DcMotorEx shootR, shootL;
    private VoltageSensor battery;

    /* ================= STATE ================= */

    private boolean enabled = false;
    private double commandedRPM = 0;
    private double stableTimer = 0;
    private long lastLoopTime;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shootR = hardwareMap.get(DcMotorEx.class, "shootr");
        shootL = hardwareMap.get(DcMotorEx.class, "shootl");

        shootL.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{shootR, shootL}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        battery = hardwareMap.voltageSensor.iterator().next();

        waitForStart();
        lastLoopTime = System.nanoTime();

        while (opModeIsActive()) {

            double dt = loopDt();

            /* ================= CONTROLS ================= */

            if (gamepad1.a) enabled = true;
            if (gamepad1.b) enabled = false;

            if (gamepad1.dpad_up) TARGET_RPM += RPM_STEP;
            if (gamepad1.dpad_down) TARGET_RPM = Math.max(0, TARGET_RPM - RPM_STEP);

            double targetRPM = (gamepad1.right_trigger > 0.1)
                    ? LOW_POWER_RPM
                    : TARGET_RPM;

            /* ================= APPLY PIDF ================= */

            double voltage = battery.getVoltage();
            double fScale = USE_VOLTAGE_COMP ? (NOMINAL_VOLTAGE / voltage) : 1.0;

            shootR.setVelocityPIDFCoefficients(kP, kI, kD, kF * fScale);
            shootL.setVelocityPIDFCoefficients(kP, kI, kD, kF * fScale);

            if (!enabled) {
                shootR.setVelocity(0);
                shootL.setVelocity(0);
                commandedRPM = 0;
                stableTimer = 0;
            } else {
                commandedRPM = USE_RAMP
                        ? ramp(commandedRPM, targetRPM, dt)
                        : targetRPM;

                double targetTPS = rpmToTicks(commandedRPM);
                shootR.setVelocity(targetTPS);
                shootL.setVelocity(targetTPS);
            }

            /* ================= READY CHECK ================= */

            double rpmR = ticksToRPM(shootR.getVelocity());
            double rpmL = ticksToRPM(shootL.getVelocity());
            double avgRPM = (rpmR + rpmL) / 2.0;

            if (Math.abs(avgRPM - targetRPM) < READY_RPM_ERROR) {
                stableTimer += dt;
            } else {
                stableTimer = 0;
            }

            boolean ready = stableTimer > READY_TIME_SEC;

            /* ================= TELEMETRY ================= */

            telemetry.addData("Enabled", enabled);
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("Commanded RPM", "%.1f", commandedRPM);
            telemetry.addData("Right RPM", "%.1f", rpmR);
            telemetry.addData("Left RPM", "%.1f", rpmL);
            telemetry.addData("Avg RPM", "%.1f", avgRPM);
            telemetry.addData("Ready To Shoot", ready ? "YES" : "NO");
            telemetry.addData("Battery", "%.2f V", voltage);
            telemetry.update();

            sleep(20);
        }
    }

    /* ================= HELPERS ================= */

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
}
