package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionOpMode;

/**
 * Simple test: Shoots ONE ball from 3 feet (36 inches) as soon as you start
 * Uses RobotActions.autoShootOneBall() - completely automatic!
 */
@Autonomous(name = "Instant Shoot Test")
public class InstantShootTest extends ActionOpMode {
    
    private Telemetry telemetryA;
    
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;
    
    private RobotActions actions;
    
    private boolean hasStarted = false;

    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize motors
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");
        
        // Initialize servos
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        reargate = hardwareMap.get(Servo.class, "reargate");
        hood1 = hardwareMap.get(Servo.class, "hood 1");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        
        // Set motor directions
        shootl.setDirection(DcMotorSimple.Direction.REVERSE);
        intakefront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeback.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Configure motors
        for (DcMotorEx m : new DcMotorEx[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        // Initialize RobotActions with voltage sensor
        actions = new RobotActions(
            intakefront, intakeback, shootr, shootl,
            launchgate, reargate, hood1, turret1, turret2,
            hardwareMap.voltageSensor.iterator().next()
        );
        
        // Reset servos
        launchgate.setPosition(0.5);
        reargate.setPosition(0.0);
        
        telemetryA.addLine("Instant Shoot Test Initialized");
        telemetryA.addLine("==============================");
        telemetryA.addLine("Will shoot ONE ball from 36 inches (3 feet)");
        telemetryA.addLine("NO turret movement - just spin & shoot");
        telemetryA.addLine("");
        telemetryA.addLine("Press START to shoot automatically!");
        telemetryA.update();
    }
    
    @Override
    public void start() {
        // As soon as you press START, shoot automatically!
        run(actions.autoShootOneBall(36));  // 36 inches = 3 feet
        hasStarted = true;
    }

    @Override
    public void loop() {
        super.loop();
        
        // Get REAL-TIME shooter velocities directly from motors
        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double vAvg = 0.5 * (vR + vL);
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avgRPM = (vAvg / 28.0) * 60.0;
        
        // Calculate target for 36 inches (3 feet)
        double targetRPM = 100.0 * 3.0 + 1150.0;  // = 1450 RPM
        double targetTPS = (targetRPM / 60.0) * 28.0;  // = 676.67 TPS
        
        // Calculate what feedforward SHOULD be
        double fContribution = 0.00084 * targetTPS;  // F × setpoint
        double kvContribution = 0.0008 * targetTPS;  // kV × setpoint  
        double ksContribution = 0.01;  // kS
        double totalFF = fContribution + kvContribution + ksContribution;
        
        telemetryA.addLine("=== INSTANT SHOOT TEST ===");
        telemetryA.addData("Status", hasStarted ? "SHOOTING!" : "Waiting for START...");
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== REAL-TIME VELOCITY ===");
        telemetryA.addData("Right Motor TPS (raw)", "%.1f", vR);
        telemetryA.addData("Left Motor TPS (raw)", "%.1f", vL);
        telemetryA.addData("Right Motor RPM", "%.0f", rpmR);
        telemetryA.addData("Left Motor RPM", "%.0f", rpmL);
        telemetryA.addData("Average RPM", "%.0f", avgRPM);
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== TARGET ===");
        telemetryA.addData("Target RPM", "%.0f", targetRPM);
        telemetryA.addData("Target TPS", "%.1f", targetTPS);
        telemetryA.addData("Error RPM", "%.0f", targetRPM - avgRPM);
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== FEEDFORWARD CALCULATION ===");
        telemetryA.addData("F × TPS", "%.4f (F=%.5f)", fContribution, 0.00084);
        telemetryA.addData("kV × TPS", "%.4f (kV=%.4f)", kvContribution, 0.0008);
        telemetryA.addData("kS", "%.4f", ksContribution);
        telemetryA.addData("Total FF", "%.4f", totalFF);
        telemetryA.addData("⚠ WARNING", totalFF > 1.0 ? "FF > 1.0!" : "FF OK");
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== SHOT INFO ===");
        telemetryA.addData("Distance", "36 inches (3.0 feet)");
        telemetryA.addLine("");
        
        telemetryA.update();
    }
    
    @Override
    public void stop() {
        intakefront.setPower(0);
        intakeback.setPower(0);
        shootr.setPower(0);
        shootl.setPower(0);
    }
}

