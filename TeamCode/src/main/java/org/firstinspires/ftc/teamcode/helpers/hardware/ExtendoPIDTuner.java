package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple; // If you need to set direction

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config // Enables FTC Dashboard configuration
@TeleOp(name = "Extendo PID Current Test") // Updated name
public class ExtendoPIDTuner extends OpMode { // Renamed class for clarity

    private PIDController pidController;

    public static double p = -0.035;
    public static double i = 0.0;
    public static double d = 0.000;
    public static int target = 0;

    public static double CURRENT_THRESHOLD_AMPS = 5.0;
    private CachingDcMotorEx extendoMotor;
    private DcMotorEx extendoMotorEx;

    @Override
    public void init() {
        // Initialize PID controller with initial values
        pidController = new PIDController(p, i, d);

        // Setup Telemetry to go to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the extendo motor
        DcMotorEx rawExtendoMotor = hardwareMap.get(DcMotorEx.class, "extendo");
        extendoMotorEx = rawExtendoMotor;
        extendoMotor = new CachingDcMotorEx(rawExtendoMotor);


        extendoMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extendoMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extendoMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Tune P, I, D, Target, and Current Threshold in FTC Dashboard."); // Removed kF from message
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update PID coefficients from FTC Dashboard in each loop
        pidController.setPID(p, i, d);

        // Get current position of the extendo motor
        int currentPosition = extendoMotor.getCurrentPosition();

        // Calculate PID output
        double pidOutput = pidController.calculate(currentPosition, target);

        // Motor power is directly from PID output
        double motorPower = pidOutput;

        // Clamp motor power to the valid range of [-1.0, 1.0]
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        // Set power to the extendo motor
        extendoMotor.setPower(motorPower);

        // --- Current Monitoring ---
        double currentAmps = extendoMotorEx.getCurrent(CurrentUnit.AMPS);
        boolean isOverCurrentDirect = extendoMotorEx.isOverCurrent();
        boolean isOverCurrentThreshold = currentAmps > CURRENT_THRESHOLD_AMPS;

        // --- Telemetry ---
        telemetry.addData("Target Position", target);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("PID Output", String.format("%.3f", pidOutput));
        // telemetry.addData("Feedforward (kF)", kF); // Removed kF from telemetry
        telemetry.addData("Calculated Motor Power", String.format("%.3f", motorPower));
        telemetry.addLine();

        telemetry.addLine("--- Extendo Motor ---");
        telemetry.addData("  Power Sent", String.format("%.3f", extendoMotor.getPower()));
        telemetry.addData("  Current (Amps)", String.format("%.2f A", currentAmps));
        telemetry.addData("  isOverCurrent() Flag", isOverCurrentDirect);
        telemetry.addData("  Over User Threshold?", isOverCurrentThreshold);

        if (isOverCurrentDirect || isOverCurrentThreshold) {
            telemetry.addLine("  !! EXTENDO OVERCURRENT WARNING !!");
        }

        telemetry.addLine();
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("Current Threshold (Amps)", String.format("%.2f A", CURRENT_THRESHOLD_AMPS));

        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure the motor is stopped
        if (extendoMotor != null) {
            extendoMotor.setPower(0);
        }
    }
}