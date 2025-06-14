package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.data.Enums.DetectedColor;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Advanced Teleop V2 Blue", group = "Examples")
public class TeleopBlue extends ActionOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private MotorControl motorControl;
    private MotorActions motorActions;

    private DetectedColor allianceColor = DetectedColor.BLUE;
    private boolean autoOuttake = true;

    // Edge-detect flags
    private boolean startPressed = false;
    private boolean xPressed = false;
    private boolean rightBumperPressed = false;
    private boolean leftBumperPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftTriggerPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadUpPressed = false;
    private boolean squarePressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    private boolean ranInit = false;

    // Track spinning
    private boolean spinActive = false;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        if (!ranInit){
            run(motorActions.safePositions());
            ranInit = true;
        }


        // Toggle auto-outtake
        if (gamepad1.start && !startPressed) {
            autoOuttake = !autoOuttake;
            startPressed = true;
        } else if (!gamepad1.start) {
            startPressed = false;
        }

        // Toggle alliance color
        if (gamepad1.x && !xPressed) {
            allianceColor = (allianceColor == DetectedColor.RED)
                    ? DetectedColor.BLUE
                    : DetectedColor.RED;
            xPressed = true;
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        // RIGHT BUMPER: always extend sample
        if (gamepad1.right_bumper && !rightBumperPressed) {
                run(motorActions.sampleExtend(300));
            rightBumperPressed = true;
        } else if (!gamepad1.right_bumper) {
            rightBumperPressed = false;
        }

        // LEFT BUMPER: grab & optional auto-outtake
        if (gamepad1.left_bumper && !leftBumperPressed) {
            run(motorActions.spin.eat());
            if (autoOuttake) {
                double extPos = motorControl.extendo.motor.getCurrentPosition();
                run(new SequentialAction(
                        new ParallelAction(
                                motorActions.outtakeTransfer(),
                                motorActions.grabUntilSample(allianceColor)
                        ),
                        motorActions.intakeTransfer(),
                        motorActions.outtakeSample(780)
                ));
            } else {
                run(new SequentialAction(
                        new ParallelAction(
                                motorActions.outtakeTransfer(),
                                motorActions.grabUntilSample(allianceColor)
                        ),
                        motorActions.intakeTransfer()
                ));
            }
            leftBumperPressed = true;
        } else if (!gamepad1.left_bumper) {
            leftBumperPressed = false;
        }

        // Color detection
        DetectedColor color = motorControl.getDetectedColor();
        boolean correctColor = color == allianceColor
                || color == DetectedColor.YELLOW
                || motorControl.lift.getTargetPosition() > 70;

        if (correctColor) {
            // RIGHT TRIGGER: outtake sample
            if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
                run(motorActions.outtakeSample());
                rightTriggerPressed = true;
            } else if (gamepad1.right_trigger == 0) {
                rightTriggerPressed = false;
            }
            // LEFT TRIGGER: outtake transfer
            if (gamepad1.left_trigger > 0 && !leftTriggerPressed) {
                run(motorActions.outtakeTransfer());
                leftTriggerPressed = true;
            } else if (gamepad1.left_trigger == 0) {
                leftTriggerPressed = false;
            }
        } else {
            // WRONG COLOR: move extendo
            if (gamepad1.right_trigger > 0) {
                int target = Math.min(motorControl.extendo.motor.getCurrentPosition() + 100, 600);
                run(new ParallelAction(
                        motorActions.extendo.set(target),
                        motorActions.extendo.waitUntilFinished()
                ));
            }
            if (gamepad1.left_trigger > 0) {
                int target = Math.max(motorControl.extendo.motor.getCurrentPosition() - 100, 40);
                run(new ParallelAction(
                        motorActions.extendo.set(target),
                        motorActions.extendo.waitUntilFinished()
                ));
            }
        }

        // D-pad down: toggle spin vs intake
        if (gamepad1.dpad_down && !dpadDownPressed) {
            if (!spinActive) {
                run(new SequentialAction(
                        motorActions.sampleExtend(200),
                        (color == allianceColor || color == DetectedColor.YELLOW)
                                ? motorActions.spin.poop()
                                : motorActions.spin.slowpoop()
                ));
                spinActive = true;
            } else {
                run(new ParallelAction(
                        motorActions.spin.stop(),
                        motorActions.intakeTransfer()
                ));
                spinActive = false;
            }
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // D-pad up: outtake sample
        if (gamepad1.dpad_up && !dpadUpPressed) {
            run(motorActions.outtakeSample());
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }


        if (gamepad1.square && !squarePressed)  {
            run(motorActions.spitSample());
            motorControl.spin.setPower(-1);
            squarePressed = true;
        } else if (!gamepad1.square) {
            squarePressed = false;
        }

        // Specimen controls
        if (gamepad1.a) {
            run(motorActions.intakeSpecimen());
        } else if (gamepad1.b) {
            run(motorActions.outtakeSpecimen());
        } else if (gamepad1.y) {
            run(motorActions.depositSpecimen());
        }

        // Hang controls
        if (gamepad1.dpad_left && !dpadLeftPressed) {
            run(motorActions.hang.up());
            dpadLeftPressed = true;
        } else if (!gamepad1.dpad_left) {
            dpadLeftPressed = false;
        }

        if (gamepad1.dpad_right && !dpadRightPressed) {
            run(motorActions.hang.down());
            dpadRightPressed = true;
        } else if (!gamepad1.dpad_right) {
            dpadRightPressed = false;
        }

        double rotation = 1;
        if (motorControl.extendo.motor.getCurrentPosition() > 50) {
            rotation = 0.5;
        }
        follower.setTeleOpMovementVectors(
                -gamepad1.right_stick_y,
                -gamepad1.right_stick_x,
                -gamepad1.left_stick_x * rotation,
                true
        );
        follower.update();
        super.loop();
        motorControl.update();


        // Telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("AutoOuttake", autoOuttake);
        telemetry.addData("Detected", color);
        telemetry.addData("ExtendoPos", motorControl.extendo.motor.getCurrentPosition());
        telemetry.addData("ExtendoVel", motorControl.extendo.motor.getVelocity());
        telemetry.addData("ExtendoReset", motorControl.extendo.resetting);
        telemetry.addData("LiftPos", motorControl.lift.motor.getCurrentPosition());
        telemetry.addData("LiftVel", motorControl.lift.motor.getVelocity());
        telemetry.addData("LiftReset", motorControl.lift.resetting);
        telemetry.update();
    }

    @Override
    public void stop() {
        // no-op
    }
}
