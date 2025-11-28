package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "AmeyPidTest", group = "Test")
@Config
public class AmeyPidTest extends OpMode {

    // ===== DASHBOARD-TUNABLE CONSTANTS =====

    // Target speed in RPM (starts at 0 so shooter is OFF until you change it)
    public static double TARGET_RPM = 0.0;

    // Feedforward + P gains (tune these on Dashboard)
    public static double kF = 0.0003;   // feedforward (normalized to 12V)
    public static double kP = 0.0002;   // proportional

    // Encoder / motor constants (change if your gear ratio is different)
    public static double TICKS_PER_REV = 28.0;   // e.g. 28 for GoBilda Yellow Jacket

    // Baseline voltage for compensation
    public static double BASELINE_VOLTAGE = 12.0;

    // ===== HARDWARE =====
    private DcMotorEx shootr;  // has encoder, positive power
    private DcMotorEx shootl;  // no encoder, just -shootr power

    private VoltageSensor batteryVoltageSensor;

    // ===== CONTROLLER STATE =====
    private ElapsedTime loopTimer = new ElapsedTime();
    private double lastErrorRPM = 0.0;

    @Override
    public void init() {
        // Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Map motors
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");

        // Run modes
        // We do ALL velocity control ourselves; we just read the encoder.
        shootr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Directions: both FORWARD, we will manually set left = -right
        shootr.setDirection(DcMotor.Direction.FORWARD);
        shootl.setDirection(DcMotor.Direction.FORWARD);

        shootr.setPower(0);
        shootl.setPower(0);

        // Get battery voltage sensor (first available)
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            batteryVoltageSensor = sensor;
            break;
        }

        loopTimer.reset();
    }

    @Override
    public void start() {
        loopTimer.reset();
        lastErrorRPM = 0;
        shootr.setPower(0);
        shootl.setPower(0);
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Get current battery voltage (fallback to baseline if something is weird)
        double batteryVoltage = (batteryVoltageSensor != null) ? batteryVoltageSensor.getVoltage() : BASELINE_VOLTAGE;
        if (batteryVoltage < 1.0) {
            batteryVoltage = BASELINE_VOLTAGE; // safety
        }

        // If target RPM is 0 or negative, shooter is OFF and controller is reset
        if (TARGET_RPM <= 0.0) {
            lastErrorRPM = 0.0;
            shootr.setPower(0.0);
            shootl.setPower(0.0);

            telemetry.addLine("Shooter OFF (TARGET_RPM <= 0)");
            telemetry.addData("batteryVolts", batteryVoltage);
            telemetry.update();
            return;
        }

        // ===== Measure velocity (in RPM) =====
        // getVelocity() works in RUN_WITHOUT_ENCODER on most REV motors/controllers
        double ticksPerSecond = shootr.getVelocity();  // encoder ticks / second
        double shooterRPM = (ticksPerSecond / TICKS_PER_REV) * 60.0;

        // ===== Error in RPM =====
        double errorRPM = TARGET_RPM - shooterRPM;

        // ===== Compute controller output in "12V-normalized power" =====
        // baseOutput is what you would send if the battery were exactly BASELINE_VOLTAGE.
        double baseOutput =
                kF * TARGET_RPM +   // feedforward (steady-state portion)
                        kP * errorRPM;      // proportional correction

        // ===== Voltage compensation =====
        // Scale the base output so the motor "feels" like it's always at BASELINE_VOLTAGE.
        double compensatedOutput = baseOutput * (BASELINE_VOLTAGE / batteryVoltage);

        // Clip to motor power range
        compensatedOutput = Range.clip(compensatedOutput, -1.0, 1.0);

        // Apply power: shootr positive, shootl negative
        shootr.setPower(compensatedOutput);
        shootl.setPower(-compensatedOutput);

        lastErrorRPM = errorRPM;

        // ===== Telemetry (for FTC Dashboard graphs) =====
        // Use simple, no-space keys so they are easy to find in the Graph tab.
        telemetry.addData("targetRpm", TARGET_RPM);
        telemetry.addData("shooterRpm", shooterRPM);
        telemetry.addData("errorRpm", errorRPM);
        telemetry.addData("baseOut", baseOutput);
        telemetry.addData("compOut", compensatedOutput);
        telemetry.addData("batteryVolts", batteryVoltage);
        telemetry.update();
    }
}
