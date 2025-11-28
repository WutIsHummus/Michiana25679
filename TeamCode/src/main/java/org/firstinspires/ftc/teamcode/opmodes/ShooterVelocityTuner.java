package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter Velocity Tuner", group = "Tuning")
public class ShooterVelocityTuner extends LinearOpMode {
    
    public static double TARGET_VELOCITY = 1000;
    public static double kV = 1.0 / 2500.0;
    public static double kA = 0.0;
    public static double kStatic = 0.0;
    
    public static boolean INTAKE_FRONT_ENABLED = true;
    public static boolean INTAKE_BACK_ENABLED = true;
    public static boolean SHOOT_ENABLED = true;
    public static boolean SHOOTL_ENABLED = true;
    
    private DcMotorEx intakeFront;
    private DcMotorEx intakeBack;
    private DcMotorEx shoot;
    private DcMotorEx shootL;
    
    private FtcDashboard dashboard;
    private ElapsedTime timer;
    
    private double lastVelocity = 0;
    private double lastTime = 0;
    
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        timer = new ElapsedTime();
        
        initializeMotors();
        
        telemetry.addLine("Shooter Velocity Tuner");
        telemetry.addLine("Use FTC Dashboard to:");
        telemetry.addLine("- Adjust TARGET_VELOCITY");
        telemetry.addLine("- Tune kV, kA, kStatic");
        telemetry.addLine("- Enable/disable motors");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A: Run motors");
        telemetry.addLine("B: Stop motors");
        telemetry.addLine("X: Measure ticks/revolution");
        telemetry.update();
        
        waitForStart();
        timer.reset();
        
        while (opModeIsActive()) {
            if (gamepad1.a) {
                runMotorsWithFeedforward();
            } else if (gamepad1.b) {
                stopAllMotors();
            } else if (gamepad1.x) {
                measureTicksPerRevolution();
            }
            
            updateTelemetry(); 
            sleep(20);
        }
        
        stopAllMotors();
    }
    
    private void initializeMotors() {
        intakeFront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeback");
        shoot = hardwareMap.get(DcMotorEx.class, "shootr");
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
        
        // Set shooter motor directions (matching teleop configuration)
        shootL.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    private void runMotorsWithFeedforward() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        if (dt == 0) dt = 0.001; // Prevent division by zero
        
        if (INTAKE_FRONT_ENABLED) {
            double currentVel = intakeFront.getVelocity();
            double acceleration = (TARGET_VELOCITY - currentVel) / dt;
            double power = calculateFeedforward(TARGET_VELOCITY, acceleration);
            intakeFront.setPower(power);
        }
        
        if (INTAKE_BACK_ENABLED) {
            double currentVel = intakeBack.getVelocity();
            double acceleration = (TARGET_VELOCITY - currentVel) / dt;
            double power = calculateFeedforward(TARGET_VELOCITY, acceleration);
            intakeBack.setPower(power);
        }
        
        if (SHOOT_ENABLED) {
            double currentVel = shoot.getVelocity();
            double acceleration = (TARGET_VELOCITY - currentVel) / dt;
            double power = calculateFeedforward(TARGET_VELOCITY, acceleration);
            shoot.setPower(power);
        }
        
        if (SHOOTL_ENABLED) {
            double currentVel = shootL.getVelocity();
            double acceleration = (TARGET_VELOCITY - currentVel) / dt;
            double power = calculateFeedforward(TARGET_VELOCITY, acceleration);
            shootL.setPower(power);
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
    
    private void measureTicksPerRevolution() {
        telemetry.addLine("MEASURING TICKS PER REVOLUTION");
        telemetry.addLine("Manually spin each motor EXACTLY 1 revolution");
        telemetry.addLine("Press A when done");
        telemetry.update();
        
        intakeFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        intakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        while (opModeIsActive() && !gamepad1.a) {
            telemetry.addData("Intake Front", intakeFront.getCurrentPosition());
            telemetry.addData("Intake Back", intakeBack.getCurrentPosition());
            telemetry.addData("Shoot", shoot.getCurrentPosition());
            telemetry.addData("ShootL", shootL.getCurrentPosition());
            telemetry.update();
        }
        
        telemetry.addLine("=== TICKS PER REVOLUTION ===");
        telemetry.addData("Intake Front", intakeFront.getCurrentPosition());
        telemetry.addData("Intake Back", intakeBack.getCurrentPosition());
        telemetry.addData("Shoot", shoot.getCurrentPosition());
        telemetry.addData("ShootL", shootL.getCurrentPosition());
        telemetry.addLine();
        telemetry.addLine("Press B to continue");
        telemetry.update();
        
        while (opModeIsActive() && !gamepad1.b) {
            sleep(50);
        }
        
        intakeFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void updateTelemetry() {
        telemetry.addData("Target Velocity", TARGET_VELOCITY);
        telemetry.addLine();
        
        telemetry.addLine("=== FEEDFORWARD ===");
        telemetry.addData("kV", kV);
        telemetry.addData("kA", kA);
        telemetry.addData("kStatic", kStatic);
        telemetry.addLine();
        
        if (INTAKE_FRONT_ENABLED) {
            double vel = intakeFront.getVelocity();
            double error = TARGET_VELOCITY - vel;
            telemetry.addLine("=== INTAKE FRONT ===");
            telemetry.addData("Velocity", "%.1f / %.1f", vel, TARGET_VELOCITY);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Power", "%.3f", intakeFront.getPower());
            telemetry.addData("Position", intakeFront.getCurrentPosition());
        }
        
        if (INTAKE_BACK_ENABLED) {
            double vel = intakeBack.getVelocity();
            double error = TARGET_VELOCITY - vel;
            telemetry.addLine("=== INTAKE BACK ===");
            telemetry.addData("Velocity", "%.1f / %.1f", vel, TARGET_VELOCITY);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Power", "%.3f", intakeBack.getPower());
            telemetry.addData("Position", intakeBack.getCurrentPosition());
        }
        
        if (SHOOT_ENABLED) {
            double vel = shoot.getVelocity();
            double error = TARGET_VELOCITY - vel;
            telemetry.addLine("=== SHOOT ===");
            telemetry.addData("Velocity", "%.1f / %.1f", vel, TARGET_VELOCITY);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Power", "%.3f", shoot.getPower());
            telemetry.addData("Position", shoot.getCurrentPosition());
        }
        
        if (SHOOTL_ENABLED) {
            double vel = shootL.getVelocity();
            double error = TARGET_VELOCITY - vel;
            telemetry.addLine("=== SHOOTL ===");
            telemetry.addData("Velocity", "%.1f / %.1f", vel, TARGET_VELOCITY);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Power", "%.3f", shootL.getPower());
            telemetry.addData("Position", shootL.getCurrentPosition());
        }
        
        telemetry.update();
    }
}
