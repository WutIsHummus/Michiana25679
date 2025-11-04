package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter Velocity Controller")
public class VelocityFinder extends LinearOpMode {

    // Motor Constants
    private static final double GEAR_RATIO = 1;
    private static final double TICKS_PER_REV = 28;
    
    // Tuned Velocity PIDF Constants - Adjustable via Dashboard
    public static double I_ZONE = 250;
    
    public static double P = 0.01;
    public static double I = 0;
    public static double D = 0.0001;
    public static double KV = 0.0008;  // Velocity feedforward (try reducing to 0.0004 or lower)
    public static double KS = 0.01;    // Static feedforward
    
    // Adjustable target RPM via Dashboard
    public static double targetRPM = 2000;

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate;
    private Servo launchgate;

    private ElapsedTime timer;
    
    // PIDF state for shooter right
    private double lastErrorR = 0;
    private double integralSumR = 0;
    private int lastEncoderPosR = 0;
    private double lastTimeR = 0;
    
    // PIDF state for shooter left
    private double lastErrorL = 0;
    private double integralSumL = 0;
    private int lastEncoderPosL = 0;
    private double lastTimeL = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");

        shootl.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeback.setDirection(DcMotorSimple.Direction.REVERSE);
        intakefront.setDirection(DcMotorSimple.Direction.REVERSE);

        reargate = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");

        launchgate.setPosition(0.5);
        
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        timer = new ElapsedTime();
        
        telemetry.addLine("Shooter Velocity Controller Initialized");
        telemetry.addLine("A = Spin up shooters");
        telemetry.addLine("B = Stop shooters");
        telemetry.addLine("Right Bumper = Run intakes inward");
        telemetry.addLine("Left Trigger = Fire launch gate");
        telemetry.addLine("Change targetRPM on FTC Dashboard");
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        while (opModeIsActive()) {
        // Drive controls
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x * 1.1;
        double rx = gamepad1.left_stick_x;
        
        double scale = 1;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        
        double flPower = (y + x + rx) / denominator * scale;
        double blPower = (y - x + rx) / denominator * scale;
        double frPower = (y - x - rx) / denominator * scale;
        double brPower = (y + x - rx) / denominator * scale;
        
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        
        // Intake control - Right Bumper to run intakes inward
        if (gamepad1.right_bumper) {
            intakefront.setPower(1.0);  // Adjust direction as needed
            intakeback.setPower(1.0);   // Adjust direction as needed
        } else {
            intakefront.setPower(0);
            intakeback.setPower(0);
        }
        
        // Shooter control
        boolean shooterOn = gamepad1.a;
        boolean shooterOff = gamepad1.b;
        
        // Always read current velocity for telemetry
        double shootrVelocity = getCurrentRPM(shootr);
        double shootlVelocity = getCurrentRPM(shootl);
        double shootrPower = 0;
        double shootlPower = 0;
        
        if (shooterOn) {
            // Control shooter right with velocity PIDF
            shootrPower = calculateVelocityPIDF(shootr, targetRPM, 
                ref -> lastErrorR = ref.error,
                ref -> integralSumR = ref.integral,
                ref -> lastEncoderPosR = ref.lastPos,
                ref -> lastTimeR = ref.lastTime,
                lastErrorR, integralSumR, lastEncoderPosR, lastTimeR);
            
            // Control shooter left with velocity PIDF
            shootlPower = calculateVelocityPIDF(shootl, targetRPM,
                ref -> lastErrorL = ref.error,
                ref -> integralSumL = ref.integral,
                ref -> lastEncoderPosL = ref.lastPos,
                ref -> lastTimeL = ref.lastTime,
                lastErrorL, integralSumL, lastEncoderPosL, lastTimeL);
            
            // Get actual power being applied
            shootrPower = shootr.getPower();
            shootlPower = shootl.getPower();
            
        } else if (shooterOff) {
            shootr.setPower(0);
            shootl.setPower(0);
            resetPIDFState();
            shootrPower = 0;
            shootlPower = 0;
        }

            // Launch gate control - Left Trigger
            if (gamepad1.left_trigger > 0.1) {
                launchgate.setPosition(0.8);
            } else {
                launchgate.setPosition(0.5);
            }
            
            // Calculate feedforward for display
            double feedforwardCalc = (KV * targetRPM) + (KS * Math.signum(targetRPM));
            
            // Telemetry
            telemetry.addData("Drive", String.format("FL:%.2f FR:%.2f BL:%.2f BR:%.2f", 
                flPower, frPower, blPower, brPower));
            telemetry.addData("", "");
            telemetry.addData("Intakes", gamepad1.right_bumper ? "RUNNING" : "STOPPED");
            telemetry.addData("Launch Gate", gamepad1.left_trigger > 0.1 ? "FIRING" : "RESET");
            telemetry.addData("", "");
            String shooterStatus = shooterOn ? "VELOCITY CONTROL" : "STOPPED";
            telemetry.addData("Shooter Status", shooterStatus);
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("", "");
            telemetry.addData("Shooter R - Current RPM", "%.0f", shootrVelocity);
            telemetry.addData("Shooter R - Power", "%.4f", shootrPower);
            telemetry.addData("Shooter R - Error", "%.0f", targetRPM - shootrVelocity);
            telemetry.addData("", "");
            telemetry.addData("Shooter L - Current RPM", "%.0f", shootlVelocity);
            telemetry.addData("Shooter L - Power", "%.4f", shootlPower);
            telemetry.addData("Shooter L - Error", "%.0f", targetRPM - shootlVelocity);
            telemetry.addData("", "");
            telemetry.addLine("=== PIDF Tuning ===");
            telemetry.addData("Feedforward", "%.4f (kV*RPM + kS)", feedforwardCalc);
            telemetry.addData("P", "%.6f", P);
            telemetry.addData("I", "%.6f", I);
            telemetry.addData("D", "%.6f", D);
            telemetry.addData("kV", "%.6f", KV);
            telemetry.addData("kS", "%.6f", KS);
            if (feedforwardCalc > 1.0) {
                telemetry.addData("⚠️ WARNING", "Feedforward > 1.0! Motor saturated!");
                telemetry.addData("Suggestion", "Reduce kV to %.6f or lower", 0.9 / targetRPM);
            }
            
            telemetry.update();
        }
        
        // Stop all motors when OpMode ends
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setPower(0);
        }
    }
    
    /**
     * Calculate velocity PIDF output for a motor
     */
    private double calculateVelocityPIDF(DcMotorEx motor, double targetRPM,
                                        java.util.function.Consumer<StateRef> errorSetter,
                                        java.util.function.Consumer<StateRef> integralSetter,
                                        java.util.function.Consumer<StateRef> posSetter,
                                        java.util.function.Consumer<StateRef> timeSetter,
                                        double lastError, double integralSum, 
                                        int lastEncoderPos, double lastTime) {
        
        // Calculate current velocity in RPM using motor.getVelocity()
        double currentVelocity = getCurrentRPM(motor);
        
        double currentTime = timer.milliseconds();
        double deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
        
        // Prevent division by zero
        if (deltaTime <= 0) {
            deltaTime = 0.02; // Default to 20ms
        }
        
        // PIDF Control
        double error = targetRPM - currentVelocity;
        
        // Integral with I_ZONE
        if (Math.abs(error) < I_ZONE) {
            integralSum += error * deltaTime;
        } else {
            integralSum = 0;
        }
        
        // Derivative
        double derivative = (error - lastError) / deltaTime;
        
        // Feedforward - convert targetRPM to motor power estimate
        // For most FTC motors, max RPM is around 5000-6000, so normalize
        double feedforward = (KV * targetRPM) + (KS * Math.signum(targetRPM));
        
        // PID
        double pidOutput = (P * error) + (I * integralSum) + (D * derivative);
        
        // Total output
        double power = feedforward + pidOutput;
        
        // Clamp power
        power = Math.max(-1.0, Math.min(1.0, power));
        
        motor.setPower(power);
        
        // Update state through setters
        StateRef ref = new StateRef();
        ref.error = error;
        errorSetter.accept(ref);
        ref.integral = integralSum;
        integralSetter.accept(ref);
        ref.lastPos = motor.getCurrentPosition();
        posSetter.accept(ref);
        ref.lastTime = currentTime;
        timeSetter.accept(ref);
        
        return power;
    }
    
    /**
     * Get current RPM of a motor
     */
    private double getCurrentRPM(DcMotorEx motor) {
        return motor.getVelocity() / (TICKS_PER_REV * GEAR_RATIO) * 60.0;
    }
    
    /**
     * Reset PIDF state for both shooters
     */
    private void resetPIDFState() {
        lastErrorR = 0;
        integralSumR = 0;
        lastEncoderPosR = shootr.getCurrentPosition();
        lastTimeR = timer.milliseconds();
        
        lastErrorL = 0;
        integralSumL = 0;
        lastEncoderPosL = shootl.getCurrentPosition();
        lastTimeL = timer.milliseconds();
    }
    
    /**
     * Helper class for passing state references
     */
    private static class StateRef {
        double error;
        double integral;
        int lastPos;
        double lastTime;
    }
}
