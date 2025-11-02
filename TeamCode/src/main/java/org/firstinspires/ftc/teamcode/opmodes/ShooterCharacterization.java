package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Config
@TeleOp(name = "Shooter Characterization", group = "Tuning")
public class ShooterCharacterization extends LinearOpMode {
    
    public static double TEST_POWER = 0.3;
    public static String MOTOR_TO_TEST = "shootr";
    
    public static double[] POWER_LEVELS = {0.3, 0.5, 0.7, 0.9};
    
    private DcMotorEx intakefront;
    private DcMotorEx intakeback;
    private DcMotorEx shootr;
    private DcMotorEx shootl;
    
    private FtcDashboard dashboard;
    private ElapsedTime timer;
    
    private boolean measuring = false;
    private int currentPowerIndex = 0;
    private double steadyStateVelocity = 0;
    
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        timer = new ElapsedTime();
        
        initializeMotors();
        
        telemetry.addLine("Shooter Characterization");
        telemetry.addLine();
        telemetry.addLine("Step 1: Find kS (static friction)");
        telemetry.addLine("  - Slowly increase TEST_POWER from 0");
        telemetry.addLine("  - Note power when motor JUST starts");
        telemetry.addLine("  - That's your kS value!");
        telemetry.addLine();
        telemetry.addLine("Step 2: Find kV (velocity gain)");
        telemetry.addLine("  - Press A to auto-test power levels");
        telemetry.addLine("  - Record power vs steady velocity");
        telemetry.addLine("  - kV = slope of (power - kS) vs velocity");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  A: Start auto characterization");
        telemetry.addLine("  B: Stop motor");
        telemetry.addLine("  X: Manual test at TEST_POWER");
        telemetry.update();
        
        waitForStart();
        timer.reset();
        
        while (opModeIsActive()) {
            if (gamepad1.a && !measuring) {
                startAutoCharacterization();
            }
            
            if (gamepad1.b) {
                stopMotor();
                measuring = false;
            }
            
            if (gamepad1.x) {
                runManualTest();
            }
            
            if (measuring) {
                runAutoCharacterization();
            }
            
            updateDashboard();
            sleep(20);
        }
        
        stopMotor();
    }
    
    private void initializeMotors() {
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");
        
        intakefront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        intakefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        intakefront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    private DcMotorEx getSelectedMotor() {
        switch (MOTOR_TO_TEST.toLowerCase()) {
            case "intakefront": return intakefront;
            case "intakeback": return intakeback;
            case "shootr": return shootr;
            case "shootl": return shootl;
            default: return shootr;
        }
    }
    
    private void startAutoCharacterization() {
        measuring = true;
        currentPowerIndex = 0;
        timer.reset();
    }
    
    private void runAutoCharacterization() {
        if (currentPowerIndex >= POWER_LEVELS.length) {
            measuring = false;
            stopMotor();
            return;
        }
        
        double power = POWER_LEVELS[currentPowerIndex];
        DcMotorEx motor = getSelectedMotor();
        motor.setPower(power);
        
        if (timer.seconds() > 2.0) {
            steadyStateVelocity = motor.getVelocity();
            currentPowerIndex++;
            timer.reset();
        }
    }
    
    private void runManualTest() {
        DcMotorEx motor = getSelectedMotor();
        motor.setPower(TEST_POWER);
    }
    
    private void stopMotor() {
        DcMotorEx motor = getSelectedMotor();
        motor.setPower(0);
    }
    
    private void updateDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        DcMotorEx motor = getSelectedMotor();
        
        double velocity = motor.getVelocity();
        double power = motor.getPower();
        
        packet.put("Motor", MOTOR_TO_TEST);
        packet.put("Power", power);
        packet.put("Velocity (TPS)", velocity);
        packet.put("TEST_POWER", TEST_POWER);
        
        if (measuring) {
            packet.put("Auto Test Progress", currentPowerIndex + "/" + POWER_LEVELS.length);
            packet.put("Current Test Power", POWER_LEVELS[Math.min(currentPowerIndex, POWER_LEVELS.length - 1)]);
            packet.put("Steady State Vel", steadyStateVelocity);
        }
        
        packet.put("Max Velocity Seen", getMaxVelocity());
        
        double estimatedKV = 0;
        if (velocity > 100) {
            estimatedKV = power / velocity;
            packet.put("Estimated kV", estimatedKV);
        }
        
        dashboard.sendTelemetryPacket(packet);
    }
    
    private double getMaxVelocity() {
        double max = 0;
        max = Math.max(max, Math.abs(intakefront.getVelocity()));
        max = Math.max(max, Math.abs(intakeback.getVelocity()));
        max = Math.max(max, Math.abs(shootr.getVelocity()));
        max = Math.max(max, Math.abs(shootl.getVelocity()));
        return max;
    }
}

