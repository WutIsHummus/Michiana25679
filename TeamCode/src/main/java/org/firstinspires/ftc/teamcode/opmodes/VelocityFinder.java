package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Shooter Velocity Controller v2")
public class VelocityFinder extends OpMode {

    // Motor Constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;
    
    // Velocity PIDF Constants - From your tuned values
    public static double p = 0.01;
    public static double i = 0.0;
    public static double d = 0.0001;
    public static double f = 0.0;     // Feedforward (F*setpoint, tune this for rapid fire!)
    public static double kV = 0.0008;  // Velocity feedforward (power per tick/s)
    public static double kS = 0.01;    // Static feedforward
    public static double I_ZONE = 250.0;  // Integrator clamp range
    
    // Adjustable target RPM via Dashboard
    public static double targetRPM = 2000;
    
    // Hood servo position
    public static double hood1Position = 0.54;

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate;
    private Servo launchgate;
    private Servo hood1;

    // PIDF Controller
    private PIDFController shooterPID;

    @Override
    public void init() {
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
        hood1 = hardwareMap.get(Servo.class, "hood 1");

        launchgate.setPosition(0.5);
        hood1.setPosition(hood1Position);
        
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Initialize PIDF controller
        shooterPID = new PIDFController(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);
        
        telemetry.addLine("Shooter Velocity Controller Initialized");
        telemetry.addLine("HOLD A = Spin up shooters (release to stop)");
        telemetry.addLine("Right Bumper = Run intakes inward");
        telemetry.addLine("Left Trigger = Fire launch gate");
        telemetry.addLine("Change targetRPM and PIDF on FTC Dashboard");
        telemetry.update();
    }

    @Override
    public void loop() {
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
        
        // Shooter control - Hold A to run, release to stop
        boolean shooterOn = gamepad1.a;
        
        // Update PIDF coefficients every loop
        shooterPID.setPIDF(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);
        
        // Convert target RPM to ticks per second
        double targetTPS = rpmToTicksPerSec(targetRPM);
        
        // Read current velocities in ticks/sec
        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        
        // Average velocity for PID control
        double vAvg = 0.5 * (vR + vL);
        
        // Convert to RPM for display
        double shootrVelocity = ticksPerSecToRPM(vR);
        double shootlVelocity = ticksPerSecToRPM(vL);
        double avgVelocityRPM = ticksPerSecToRPM(vAvg);
        
        double shooterPower = 0;
        double pidOutput = 0;
        double feedforward = 0;
        
        if (shooterOn) {
            // PID on average velocity (in ticks/sec)
            pidOutput = shooterPID.calculate(vAvg, targetTPS);
            
            // Feedforward
            double sgn = Math.signum(targetTPS);
            feedforward = (Math.abs(targetTPS) > 1e-6) ? (kS * sgn + kV * targetTPS) : 0.0;
            
            // Total power
            shooterPower = pidOutput + feedforward;
            
            // Safety: If already at or above target, reduce power to prevent overshoot
            if (avgVelocityRPM >= targetRPM && shooterPower > 0) {
                shooterPower = Math.min(shooterPower, 0.5); // Cap at 50% if at target
            }
            
            // Clamp to motor limits
            shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
            
            // Apply same power to both motors
            shootr.setPower(shooterPower);
            shootl.setPower(shooterPower);
            
        } else {
            // Not pressing A - turn off motors
            shootr.setPower(0);
            shootl.setPower(0);
            shooterPID.reset();
            shooterPower = 0;
            pidOutput = 0;
            feedforward = 0;
        }

            // Hood position - continuously update from dashboard
            hood1.setPosition(hood1Position);
            
            // Launch gate control - Left Trigger
            if (gamepad1.left_trigger > 0.1) {
                launchgate.setPosition(0.8);
            } else {
                launchgate.setPosition(0.5);
            }
            
            // Telemetry
            telemetry.addData("Drive", String.format("FL:%.2f FR:%.2f BL:%.2f BR:%.2f", 
                flPower, frPower, blPower, brPower));
            telemetry.addData("", "");
            telemetry.addData("Intakes", gamepad1.right_bumper ? "RUNNING" : "STOPPED");
            telemetry.addData("Launch Gate", gamepad1.left_trigger > 0.1 ? "FIRING" : "RESET");
            telemetry.addData("Hood1 Position", "%.2f", hood1Position);
            telemetry.addData("", "");
            
            String shooterStatus = shooterOn ? "VELOCITY CONTROL" : "STOPPED";
            telemetry.addData("Shooter Status", shooterStatus);
            telemetry.addData("", "");
            telemetry.addLine("=== TARGET (Updates Live from Dashboard) ===");
            telemetry.addData("Target RPM", "%.0f ← Change on dashboard!", targetRPM);
            telemetry.addData("Target TPS", "%.1f (auto-calculated)", targetTPS);
            telemetry.addData("Will go to", "%.0f RPM when you press A", targetRPM);
            telemetry.addData("", "");
            
            telemetry.addLine("--- Right Motor ---");
            telemetry.addData("vR (TPS)", "%.1f", vR);
            telemetry.addData("vR (RPM)", "%.0f", shootrVelocity);
            
            telemetry.addLine("--- Left Motor ---");
            telemetry.addData("vL (TPS)", "%.1f", vL);
            telemetry.addData("vL (RPM)", "%.0f", shootlVelocity);
            
            telemetry.addLine("--- Controller ---");
            telemetry.addData("vAvg (TPS)", "%.1f", vAvg);
            telemetry.addData("vAvg (RPM)", "%.0f", avgVelocityRPM);
            telemetry.addData("Error (TPS)", "%.1f", targetTPS - vAvg);
            telemetry.addData("Error (RPM)", "%.0f", targetRPM - avgVelocityRPM);
            telemetry.addData("", "");
            telemetry.addData("PID Output", "%.4f", pidOutput);
            telemetry.addData("Feedforward", "%.4f (kS=%.4f + kV*TPS=%.4f)", feedforward, kS * Math.signum(targetTPS), kV * targetTPS);
            telemetry.addData("Total Power", "%.4f", shooterPower);
            
            telemetry.addData("", "");
            telemetry.addLine("=== DEBUG: RAW VALUES ===");
            telemetry.addData("shootr.getVelocity()", "%.1f TPS (raw)", vR);
            telemetry.addData("shootl.getVelocity()", "%.1f TPS (raw)", vL);
            telemetry.addData("TICKS_PER_REV", "%.0f", TICKS_PER_REV);
            telemetry.addData("GEAR_RATIO", "%.2f", GEAR_RATIO);
            
            telemetry.addData("", "");
            telemetry.addLine("=== PIDF Constants (FROM DASHBOARD) ===");
            telemetry.addData("p", "%.6f", p);
            telemetry.addData("i", "%.6f", i);
            telemetry.addData("d", "%.6f", d);
            telemetry.addData("f", "%.6f <-- NEW: F*setpoint for rapid fire!", f);
            telemetry.addData("kV", "%.6f", kV);
            telemetry.addData("kS", "%.6f", kS);
            telemetry.addData("I_ZONE", "%.1f", I_ZONE);
            telemetry.addData("F contribution", "%.4f (F*targetTPS)", f * targetTPS);
            telemetry.addData("", "");
            telemetry.addData("kV * targetTPS", "%.4f (should be ~0.84)", kV * targetTPS);
            
            if (shooterPower >= 0.99 || shooterPower <= -0.99) {
                telemetry.addData("⚠️ SATURATED!", "Power capped at 1.0!");
                telemetry.addData("", "Reduce kV to %.4f or lower", 0.8 / targetTPS);
            }
            
            telemetry.addData("", "");
            telemetry.addLine("Tuning Guide (for TPS units):");
            telemetry.addData("1. F", "Tune F (0.0008-0.001) for base velocity (rapid fire!)");
            telemetry.addData("2. kV + kS", "Or use kV/kS instead (alternative method)");
            telemetry.addData("3. P", "Add 0.0003-0.0008 for error correction");
            telemetry.addData("4. D", "Add small D for stability if needed");
            telemetry.addData("Note", "F term helps maintain speed during rapid fire!");
            
            telemetry.update();
    }
    
    @Override
    public void stop() {
        shootr.setPower(0);
        shootl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        intakefront.setPower(0);
        intakeback.setPower(0);
    }
    
    /**
     * Convert RPM to ticks per second
     */
    private static double rpmToTicksPerSec(double rpm) {
        double motorRPM = rpm * GEAR_RATIO;
        return (motorRPM / 60.0) * TICKS_PER_REV;
    }

    /**
     * Convert ticks per second to RPM
     */
    private static double ticksPerSecToRPM(double tps) {
        double motorRPM = (tps / TICKS_PER_REV) * 60.0;
        return motorRPM / GEAR_RATIO;
    }
}
