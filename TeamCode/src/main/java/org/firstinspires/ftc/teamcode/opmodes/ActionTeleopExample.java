package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionOpMode;

@TeleOp(name = "Action Teleop Example", group = "Examples")
public class ActionTeleopExample extends ActionOpMode {
    
    private DcMotor fl, fr, bl, br;
    private DcMotor intakefront, intakeback;
    private DcMotor shootr, shootl;
    private Servo reargate, launchgate;
    
    private RobotActions actions;
    
    private boolean ranInit = false;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;
    private boolean rightBumperPressed = false;
    private boolean leftBumperPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftTriggerPressed = false;
    
    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "frontleft");
        fr = hardwareMap.get(DcMotor.class, "frontright");
        bl = hardwareMap.get(DcMotor.class, "backleft");
        br = hardwareMap.get(DcMotor.class, "backright");
        
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        
        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback = hardwareMap.get(DcMotor.class, "intakeback");
        shootr = hardwareMap.get(DcMotor.class, "shootr");
        shootl = hardwareMap.get(DcMotor.class, "shootl");
        
        reargate = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        
        actions = new RobotActions(intakefront, intakeback, shootr, shootl, launchgate, reargate);
        
        telemetry.addLine("Action Teleop Example Initialized");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        if (!ranInit) {
            run(actions.safePositions());
            ranInit = true;
        }
        
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
        
        if (gamepad1.right_bumper && !rightBumperPressed) {
            run(actions.startIntake());
            rightBumperPressed = true;
        } else if (!gamepad1.right_bumper) {
            rightBumperPressed = false;
        }
        
        if (gamepad1.left_bumper && !leftBumperPressed) {
            run(actions.stopIntake());
            leftBumperPressed = true;
        } else if (!gamepad1.left_bumper) {
            leftBumperPressed = false;
        }
        
        if (gamepad1.right_trigger > 0.1 && !rightTriggerPressed) {
            run(actions.shootSequence());
            rightTriggerPressed = true;
        } else if (gamepad1.right_trigger == 0) {
            rightTriggerPressed = false;
        }
        
        if (gamepad1.left_trigger > 0.1 && !leftTriggerPressed) {
            run(actions.rapidFire());
            leftTriggerPressed = true;
        } else if (gamepad1.left_trigger == 0) {
            leftTriggerPressed = false;
        }
        
        if (gamepad1.a && !aPressed) {
            run(actions.shooter.spinUp());
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }
        
        if (gamepad1.b && !bPressed) {
            run(actions.shooter.stop());
            bPressed = true;
        } else if (!gamepad1.b) {
            bPressed = false;
        }
        
        if (gamepad1.x && !xPressed) {
            run(actions.launch.fire());
            xPressed = true;
        } else if (!gamepad1.x) {
            xPressed = false;
        }
        
        if (gamepad1.y && !yPressed) {
            run(actions.launch.reset());
            yPressed = true;
        } else if (!gamepad1.y) {
            yPressed = false;
        }
        
        super.loop();
        
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.addData("Drive", String.format("FL:%.2f FR:%.2f BL:%.2f BR:%.2f", 
            flPower, frPower, blPower, brPower));
        telemetry.update();
    }
    
    @Override
    public void stop() {
        run(actions.safePositions());
    }
}

