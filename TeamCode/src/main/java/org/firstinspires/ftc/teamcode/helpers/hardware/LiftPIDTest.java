package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@TeleOp(name = "Lift PID Current Test") // Renamed for clarity
public class LiftPIDTest extends OpMode { // Renamed class

    private PIDController pidController;

    public static double p = 0, i = 0, d = 0;
    public static int target = 0; // Target position for the lift
    public static double GRAVITY_FEEDFORWARD = 0;

    public static double CURRENT_THRESHOLD_AMPS = 5.0; // Example, tune this!

    private CachingDcMotorEx motor1, motor2;
    private DcMotorEx motor1Ex, motor2Ex;


    @Override
    public void init() {
        pidController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo ptor,ptol;
        ptor           = hardwareMap.get(Servo.class, "ptor");
        ptol           = hardwareMap.get(Servo.class, "ptol");
        ptol.setPosition(0.44);
        ptor.setPosition(0.60);


        DcMotorEx rawMotor1 = hardwareMap.get(DcMotorEx.class, "liftr");
        DcMotorEx rawMotor2 = hardwareMap.get(DcMotorEx.class, "liftl");

        motor1 = new CachingDcMotorEx(rawMotor1);
        motor2 = new CachingDcMotorEx(rawMotor2);

        motor1Ex = rawMotor1; // Store for current methods
        motor2Ex = rawMotor2; // Store for current methods


        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Optional: Reset encoders if you want to start from a known zero position
        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized. Tune P, I, D, Target, and Current Threshold in FTC Dashboard.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update PID constants from FTC Dashboard
        pidController.setPID(p,i,d);

        // Get current position (assuming motor1 is the primary encoder source)
        int currentPosition = motor1.getCurrentPosition();

        // Calculate PID output
        double pidOutput = pidController.calculate(currentPosition, target);

        double motorPower = pidOutput + GRAVITY_FEEDFORWARD;


        // Clamp motor power to [-1, 1]
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        // Set power to motors
        motor1.setPower(motorPower);
        motor2.setPower(motorPower);

        // --- Current Monitoring ---
        double motor1CurrentAmps = motor1Ex.getCurrent(CurrentUnit.AMPS);
        double motor2CurrentAmps = motor2Ex.getCurrent(CurrentUnit.AMPS);

        boolean motor1IsOverCurrentDirect = motor1Ex.isOverCurrent();
        boolean motor2IsOverCurrentDirect = motor2Ex.isOverCurrent();

        boolean motor1IsOverThreshold = motor1CurrentAmps > CURRENT_THRESHOLD_AMPS;
        boolean motor2IsOverThreshold = motor2CurrentAmps > CURRENT_THRESHOLD_AMPS;

        // --- Telemetry ---
        telemetry.addData("Target Position", target);
        telemetry.addData("Current Position (motor1)", currentPosition);
        telemetry.addData("PID Output", String.format("%.3f", pidOutput));
        telemetry.addData("Gravity Feedforward", GRAVITY_FEEDFORWARD);
        telemetry.addData("Calculated Motor Power", String.format("%.3f", motorPower));
        telemetry.addLine();

        telemetry.addLine("--- Motor 1 (liftr) ---");
        telemetry.addData("  M1 Power Sent", String.format("%.3f", motor1.getPower()));
        telemetry.addData("  M1 Current (Amps)", String.format("%.2f A", motor1CurrentAmps));
        telemetry.addData("  M1 isOverCurrent()", motor1IsOverCurrentDirect);
        telemetry.addData("  M1 Over Threshold?", motor1IsOverThreshold);
        if (motor1IsOverCurrentDirect || motor1IsOverThreshold) {
            telemetry.addLine("  !! M1 OVERCURRENT WARNING !!");
        }

        telemetry.addLine();
        telemetry.addLine("--- Motor 2 (liftl) ---");
        telemetry.addData("  M2 Power Sent", String.format("%.3f", motor2.getPower()));
        telemetry.addData("  M2 Current (Amps)", String.format("%.2f A", motor2CurrentAmps));
        telemetry.addData("  M2 isOverCurrent()", motor2IsOverCurrentDirect);
        telemetry.addData("  M2 Over Threshold?", motor2IsOverThreshold);
        if (motor2IsOverCurrentDirect || motor2IsOverThreshold) {
            telemetry.addLine("  !! M2 OVERCURRENT WARNING !!");
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
        // Ensure motors are stopped when OpMode ends
        if (motor1 != null) {
            motor1.setPower(0);
        }
        if (motor2 != null) {
            motor2.setPower(0);
        }
    }
}