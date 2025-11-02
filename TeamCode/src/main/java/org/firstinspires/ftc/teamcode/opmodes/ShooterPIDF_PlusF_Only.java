package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "ShooterDualPF", group = "Test")
public class ShooterDualPF extends LinearOpMode {

    // ===== Dashboard-tunable constants =====
    public static double GEAR_RATIO = 1.0;             // 1.0 for bare motor, else 13.7, 19.2, etc.
    public static double TICKS_PER_REV_MOTOR = 28.0;   // GoBILDA bare motor encoder
    public static double TARGET_RPS = 70.0;            // target flywheel speed (rot/s)
    public static double SHOOT_POWER = 0.67;           // manual power control (both motors)
    public static boolean USE_VELOCITY_CONTROL = true; // true = PIDF velocity, false = raw power

    // PIDF constants (P + F only)
    public static double kP = 0.0005;
    public static double kF = 1.0 / 5000.0;            // feedforward (power per ticks/sec)
    public static double kS = 0.0;                     // optional static offset
    public static boolean VOLTAGE_COMP = true;
    public static double NOMINAL_VOLTAGE = 12.0;

    private DcMotorEx shootr, shootl;

    @Override
    public void runOpMode() {
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");

        shootr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready — both shooter motors linked.").update();

        waitForStart();

        while (opModeIsActive()) {

            // --- voltage-compensated F ---
            double vBat = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double kF_eff = kF * (VOLTAGE_COMP ? (NOMINAL_VOLTAGE / Math.max(10.0, vBat)) : 1.0);
            shootr.setVelocityPIDFCoefficients(kP, 0.0, 0.0, kF_eff);

            double ticksPerRev = TICKS_PER_REV_MOTOR * GEAR_RATIO;
            double targetTPS = TARGET_RPS * ticksPerRev;

            // optional static offset
            if (kS != 0.0 && kF_eff != 0.0) {
                double sign = Math.signum(TARGET_RPS);
                targetTPS += (kS / kF_eff) * sign * ticksPerRev;
            }

            if (USE_VELOCITY_CONTROL) {
                // Velocity PIDF control
                shootr.setVelocity(targetTPS);
                shootl.setVelocity(-targetTPS); // opposite direction
            } else {
                // Open-loop power control
                shootr.setPower(SHOOT_POWER);
                shootl.setPower(-SHOOT_POWER);
            }

            // --- Telemetry + Dashboard graph data ---
            double r_ticksPerSec = shootr.getVelocity();
            double l_ticksPerSec = shootl.getVelocity();
            double r_rps = r_ticksPerSec / ticksPerRev;
            double l_rps = l_ticksPerSec / ticksPerRev;

            telemetry.addData("Mode", USE_VELOCITY_CONTROL ? "Velocity (P+F)" : "Raw Power");
            telemetry.addData("Target RPS", TARGET_RPS);
            telemetry.addData("Shootr RPS", r_rps);
            telemetry.addData("Shootl RPS", l_rps);
            telemetry.addData("Power", SHOOT_POWER);
            telemetry.addData("kP", kP);
            telemetry.addData("kF", kF);
            telemetry.update();
        }

        shootr.setPower(0);
        shootl.setPower(0);
    }
}



