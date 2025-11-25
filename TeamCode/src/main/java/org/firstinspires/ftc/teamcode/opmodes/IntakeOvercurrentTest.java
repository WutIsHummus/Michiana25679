package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
@TeleOp(name = "Intake Overcurrent", group = "Hardware Tests")
public class IntakeOvercurrentTest extends LinearOpMode {
    
    private DcMotorEx intakeMotor = null;
    
    private double currentThreshold = 5.0;
    private int overcurrentCount = 0;
    
    private boolean xPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    
    @Override
    public void runOpMode() {
        initializeHardware();
        
        waitForStart();
        
        while (opModeIsActive()) {
            controlIntakeMotor();
            handleButtons();
            checkOvercurrent();
            sleep(50);
        }
        
        intakeMotor.setPower(0);
    }
    
    private void initializeHardware() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void controlIntakeMotor() {
        double motorPower = 0;
        
        if (gamepad1.left_trigger > 0.1) {
            motorPower = gamepad1.left_trigger;
        }
        else if (gamepad1.right_trigger > 0.1) {
            motorPower = -gamepad1.right_trigger;
        }
        
        intakeMotor.setPower(motorPower);
    }
    
    private void handleButtons() {
        if (gamepad1.x && !xPressed) {
            overcurrentCount = 0;
            gamepad1.rumble(200);
            xPressed = true;
        } else if (!gamepad1.x) {
            xPressed = false;
        }
        
        if (gamepad1.dpad_up && !dpadUpPressed) {
            currentThreshold = Math.min(currentThreshold + 0.5, 10.0);
            gamepad1.rumble(50);
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }
        
        if (gamepad1.dpad_down && !dpadDownPressed) {
            currentThreshold = Math.max(currentThreshold - 0.5, 1.0);
            gamepad1.rumble(50);
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }
    }
    
    private boolean checkOvercurrent() {
        double currentAmps = intakeMotor.getCurrent(CurrentUnit.AMPS);
        boolean isOverCurrent = currentAmps > currentThreshold;
        
        if (isOverCurrent) {
            overcurrentCount++;
            intakeMotor.setPower(0.0);
            return true;
        }
        
        return false;
    }
}
