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

import org.firstinspires.ftc.teamcode.helpers.hardware.VelocityPIDFController;

@Disabled
@Config
@TeleOp(name = "Shooter Velocity PIDF", group = "Tuning")
public class ShooterVelocityPIDF extends LinearOpMode {
    
    public static double TARGET_VELOCITY = 4200;
    
    public static double kS = 0.05;
    public static double kV = 0.0002;
    public static double kA = 0.0;
    
    public static double kP = 0.0005;
    public static double kI = 0.0;
    public static double kD = 0.0001;
    
    public static boolean RUN_INTAKEFRONT = true;
    public static boolean RUN_INTAKEBACK = true;
    public static boolean RUN_SHOOTR = true;
    public static boolean RUN_SHOOTL = true;
    
    private DcMotorEx intakefront;
    private DcMotorEx intakeback;
    private DcMotorEx shootr;
    private DcMotorEx shootl;
    
    private VelocityPIDFController ctrlIntakeFront;
    private VelocityPIDFController ctrlIntakeBack;
    private VelocityPIDFController ctrlShootR;
    private VelocityPIDFController ctrlShootL;
    
    private FtcDashboard dashboard;
    private ElapsedTime timer;
    
    private boolean running = false;
    private boolean lastA = false;
    private boolean lastB = false;
    
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        timer = new ElapsedTime();
        
        initializeMotors();
        initializeControllers();
        
        waitForStart();
        timer.reset();
        
        while (opModeIsActive()) {
            updateControllerGains();
            
            if (gamepad1.a && !lastA) {
                running = !running;
                if (running) {
                    resetAllControllers();
                } else {
                    stopAllMotors();
                }
            }
            lastA = gamepad1.a;
            
            if (gamepad1.b && !lastB) {
                stopAllMotors();
                running = false;
            }
            lastB = gamepad1.b;
            
            if (gamepad1.dpad_up) TARGET_VELOCITY += 100;
            if (gamepad1.dpad_down) TARGET_VELOCITY -= 100;
            
            if (running) {
                runVelocityControl();
            }
            
            updateDashboard();
            sleep(10);
        }
        
        stopAllMotors();
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
        
        intakefront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        intakefront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    private void initializeControllers() {
        ctrlIntakeFront = new VelocityPIDFController();
        ctrlIntakeBack = new VelocityPIDFController();
        ctrlShootR = new VelocityPIDFController();
        ctrlShootL = new VelocityPIDFController();
    }
    
    private void updateControllerGains() {
        VelocityPIDFController[] controllers = {ctrlIntakeFront, ctrlIntakeBack, ctrlShootR, ctrlShootL};
        for (VelocityPIDFController ctrl : controllers) {
            ctrl.kS = kS;
            ctrl.kV = kV;
            ctrl.kA = kA;
            ctrl.kP = kP;
            ctrl.kI = kI;
            ctrl.kD = kD;
        }
    }
    
    private void resetAllControllers() {
        ctrlIntakeFront.reset();
        ctrlIntakeBack.reset();
        ctrlShootR.reset();
        ctrlShootL.reset();
    }
    
    private void runVelocityControl() {
        if (RUN_INTAKEFRONT) {
            double vel = intakefront.getVelocity();
            double power = ctrlIntakeFront.update(TARGET_VELOCITY, vel);
            intakefront.setPower(power);
        }
        
        if (RUN_INTAKEBACK) {
            double vel = intakeback.getVelocity();
            double power = ctrlIntakeBack.update(TARGET_VELOCITY, vel);
            intakeback.setPower(power);
        }
        
        if (RUN_SHOOTR) {
            double vel = shootr.getVelocity();
            double power = ctrlShootR.update(TARGET_VELOCITY, vel);
            shootr.setPower(power);
        }
        
        if (RUN_SHOOTL) {
            double vel = shootl.getVelocity();
            double power = ctrlShootL.update(TARGET_VELOCITY, vel);
            shootl.setPower(power);
        }
    }
    
    private void stopAllMotors() {
        intakefront.setPower(0);
        intakeback.setPower(0);
        shootr.setPower(0);
        shootl.setPower(0);
    }
    
    private void updateDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        
        packet.put("Running", running);
        packet.put("Time", timer.seconds());
        packet.put("Target Velocity", TARGET_VELOCITY);
        
        packet.put("kS", kS);
        packet.put("kV", kV);
        packet.put("kA", kA);
        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);
        
        if (RUN_INTAKEFRONT) {
            double vel = intakefront.getVelocity();
            packet.put("IntakeFront Velocity", vel);
            packet.put("IntakeFront Error", TARGET_VELOCITY - vel);
            packet.put("IntakeFront Power", intakefront.getPower());
        }
        
        if (RUN_INTAKEBACK) {
            double vel = intakeback.getVelocity();
            packet.put("IntakeBack Velocity", vel);
            packet.put("IntakeBack Error", TARGET_VELOCITY - vel);
            packet.put("IntakeBack Power", intakeback.getPower());
        }
        
        if (RUN_SHOOTR) {
            double vel = shootr.getVelocity();
            packet.put("ShootR Velocity", vel);
            packet.put("ShootR Error", TARGET_VELOCITY - vel);
            packet.put("ShootR Power", shootr.getPower());
        }
        
        if (RUN_SHOOTL) {
            double vel = shootl.getVelocity();
            packet.put("ShootL Velocity", vel);
            packet.put("ShootL Error", TARGET_VELOCITY - vel);
            packet.put("ShootL Power", shootl.getPower());
        }
        
        dashboard.sendTelemetryPacket(packet);
    }
}

