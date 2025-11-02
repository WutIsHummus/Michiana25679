package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "ShooterPF_Tuner", group = "Test")
public class ShooterPF_Tuner extends LinearOpMode {

    // ---- Mechanism / units ----
    public static double TICKS_PER_REV_MOTOR = 28.0; // GoBILDA bare motor
    public static double GEAR_RATIO = 1.0;           // set if you have gearing on the flywheel

    // ---- Targets ----
    public static double TARGET_RPS = 70.0;          // desired flywheel speed (rot/s)

    // ---- Gains (P + F only; no I/D) ----
    // 12V-normalized feedforward measured by your calibrator:
    public static double kF_12V = 0.00044494;
    public static double kP     = 0.0005;            // start small; raise until recovery is snappy but stable
    public static double kS     = 0.00;              // optional static offset (0.03–0.07 if startup is sticky)
    public static double NOMINAL_VOLTAGE = 12.0;

    // ---- Tuning helpers ----
    public static boolean TUNING_F_ONLY = false;     // set true to tune F: forces P=0 and disables boost
    public static boolean USE_BOOST = false;         // optional first-spin target bump (still P+F)
    public static double BOOST_FRAC = 0.06;          // +6% target during boost window
    public static int BOOST_TIME_MS  = 300;

    // ---- Recovery timing params ----
    public static double BAND_FRAC = 0.02;           // ±2% band defines "recovered"
    public static double DIP_FRAC  = 0.95;           // dip threshold: below 95% = shot detected

    private DcMotorEx shootr, shootl;
    private VoltageSensor vSense;

    // simple state container for recovery timing
    static class RecState {
        boolean spunUp;        // we have reached band at least once
        boolean dipped;        // we have seen a dip below 95%
        boolean recovered;     // we returned to ±band after dip
        double dipTimeS;       // time when dip started
        int recoveryMs = -1;   // last measured recovery time
        void reset() { spunUp = dipped = recovered = false; dipTimeS = 0; recoveryMs = -1; }
    }
    private final RecState rec = new RecState();
    private double startTimeS;

    @Override
    public void runOpMode() {
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");

        // voltage sensor (fallback-safe)
        vSense = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        // allow full encoder rate (REV quirk)
        MotorConfigurationType tr = shootr.getMotorType();
        tr.setAchieveableMaxRPMFraction(1.0);
        shootr.setMotorType(tr);
        MotorConfigurationType tl = shootl.getMotorType();
        tl.setAchieveableMaxRPMFraction(1.0);
        shootl.setMotorType(tl);

        // reset & run with encoder (we read velocity, command power ourselves)
        shootr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Shooter P+F tuner: graph Shootr RPS, Target RPS, Recovery (ms)");

        waitForStart();

        rec.reset();
        startTimeS = getRuntime();
        final double ticksPerRev = TICKS_PER_REV_MOTOR * GEAR_RATIO;

        while (opModeIsActive()) {
            // ---- build 12V-normalized F at current voltage ----
            double vBat = (vSense != null) ? vSense.getVoltage() : NOMINAL_VOLTAGE;
            double kF_eff = kF_12V * (NOMINAL_VOLTAGE / Math.max(10.0, vBat));

            // during F-only tuning, kill P so F actually moves RPS
            double kP_use = TUNING_F_ONLY ? 0.0 : kP;

            // target TPS (with optional brief boost on first spin)
            double targetRps = TARGET_RPS;
            if (USE_BOOST && !TUNING_F_ONLY) {
                int msSinceStart = (int) ((getRuntime() - startTimeS) * 1000);
                if (msSinceStart < BOOST_TIME_MS) {
                    targetRps *= (1.0 + BOOST_FRAC);
                }
            }
            double targetTPS = targetRps * ticksPerRev;

            // optional kS as tiny bias in TPS units
            if (kS != 0.0 && kF_eff != 0.0) {
                double sign = Math.signum(targetRps);
                targetTPS += (kS / kF_eff) * sign * ticksPerRev;
            }

            // ---- read current velocity (ticks/sec) ----
            double rTPS = shootr.getVelocity();
            double lTPS = shootl.getVelocity();
            double errorTPS = targetTPS - rTPS;

            // ---- P + F command (custom loop; no SDK PIDF needed) ----
            double uFF = kF_eff * targetTPS;
            double uP  = kP_use * errorTPS;
            double powerR = Range.clip(uFF + uP, -1.0, 1.0);
            double powerL = -powerR; // mirror

            shootr.setPower(1.0);
            shootl.setPower(-1.0);


            // ---- Recovery timer (dip <95%, recover to ±2%) ----
            double rps = rTPS / ticksPerRev;
            boolean inBand = Math.abs(rps - TARGET_RPS) <= BAND_FRAC * TARGET_RPS;

            if (!rec.spunUp) {
                if (inBand) rec.spunUp = true; // seen nominal once
            } else if (!rec.dipped) {
                if (rps < DIP_FRAC * TARGET_RPS) {
                    rec.dipped = true;
                    rec.recovered = false;
                    rec.dipTimeS = getRuntime();
                }
            } else if (!rec.recovered) {
                if (inBand) {
                    rec.recovered = true;
                    rec.recoveryMs = (int) ((getRuntime() - rec.dipTimeS) * 1000.0);
                }
            }
            // You can reset timing by toggling target or pressing B:
            if (gamepad1.b) rec.reset();

            // ---- Telemetry / Dashboard ----
            telemetry.addData("Mode", "P+F custom loop");
            telemetry.addData("kF_12V", kF_12V);
            telemetry.addData("kF_eff", kF_eff);
            telemetry.addData("kP_use", kP_use);
            telemetry.addData("Target RPS", TARGET_RPS);
            telemetry.addData("Shootr RPS", rps);
            telemetry.addData("Shootl RPS", lTPS / ticksPerRev);
            telemetry.addData("FF power", uFF);
            telemetry.addData("P power", uP);
            telemetry.addData("Out powerR", powerR);
            telemetry.addData("vBat", "%.2f", vBat);
            telemetry.addData("Recovery (ms)", rec.recoveryMs);
            telemetry.update();
        }

        shootr.setPower(0);
        shootl.setPower(0);
    }
}



