package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter Velocity Graph", group = "Tuning")
public class ShooterVelocityGraph extends LinearOpMode {
    
    public static double TARGET_VELOCITY = 2000;
    public static double kV = 1.0 / 2500.0;
    public static double kA = 0.001;
    public static double kStatic = 0.05;
    
    public static String MOTOR_TO_GRAPH = "shoot";
    
    private DcMotorEx intakeFront;
    private DcMotorEx intakeBack;
    private DcMotorEx shoot;
    private DcMotorEx shootL;
    
    private FtcDashboard dashboard;
    private ElapsedTime timer;
    
    private double lastVelocity = 0;
    private double lastTime = 0;
    
    private boolean running = false;
    
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        timer = new ElapsedTime();
        
        initializeMotors();
        
        waitForStart();
        timer.reset();
        lastTime = 0;
        
        while (opModeIsActive()) {
            if (gamepad1.a) {
                running = true;
            } else if (gamepad1.b) {
                running = false;
                stopAllMotors();
            }
            
            if (running) {
                runMotorsWithFeedforward();
            }
            
            updateGraph();
            sleep(10);
        }
        
        stopAllMotors();
    }
    
    private void initializeMotors() {
        intakeFront = hardwareMap.get(DcMotorEx.class, "Intake front");
        intakeBack = hardwareMap.get(DcMotorEx.class, "Intake back");
        shoot = hardwareMap.get(DcMotorEx.class, "shoot");
        shootL = hardwareMap.get(DcMotorEx.class, "shootl");
        
        intakeFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        intakeFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        intakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    private void runMotorsWithFeedforward() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        
        if (dt > 0) {
            DcMotorEx[] motors = {intakeFront, intakeBack, shoot, shootL};
            
            for (DcMotorEx motor : motors) {
                double currentVel = motor.getVelocity();
                double acceleration = (TARGET_VELOCITY - currentVel) / dt;
                double power = calculateFeedforward(TARGET_VELOCITY, acceleration);
                motor.setPower(power);
            }
        }
        
        lastTime = currentTime;
    }
    
    private double calculateFeedforward(double targetVelocity, double acceleration) {
        double power = kV * targetVelocity + kA * acceleration + kStatic;
        return Math.max(-1.0, Math.min(1.0, power));
    }
    
    private void stopAllMotors() {
        intakeFront.setPower(0);
        intakeBack.setPower(0);
        shoot.setPower(0);
        shootL.setPower(0);
    }
    
    private void updateGraph() {
        TelemetryPacket packet = new TelemetryPacket();
        
        DcMotorEx selectedMotor = getSelectedMotor();
        
        if (selectedMotor != null) {
            double currentVel = selectedMotor.getVelocity();
            double error = TARGET_VELOCITY - currentVel;
            double power = selectedMotor.getPower();
            
            packet.put("Target Velocity", TARGET_VELOCITY);
            packet.put("Current Velocity", currentVel);
            packet.put("Velocity Error", error);
            packet.put("Motor Power", power);
            packet.put("kV", kV);
            packet.put("kA", kA);
            packet.put("kStatic", kStatic);
            
            packet.put("Intake Front Vel", intakeFront.getVelocity());
            packet.put("Intake Back Vel", intakeBack.getVelocity());
            packet.put("Shoot Vel", shoot.getVelocity());
            packet.put("ShootL Vel", shootL.getVelocity());
            
            packet.put("Running", running);
            packet.put("Time", timer.seconds());
        }
        
        dashboard.sendTelemetryPacket(packet);
    }
    
    private DcMotorEx getSelectedMotor() {
        switch (MOTOR_TO_GRAPH.toLowerCase()) {
            case "intake front":
            case "intakefront":
                return intakeFront;
            case "intake back":
            case "intakeback":
                return intakeBack;
            case "shoot":
                return shoot;
            case "shootl":
                return shootL;
            default:
                return shoot;
        }
    }
}

