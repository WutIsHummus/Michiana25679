package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.data.Enums.DetectedColor;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "headinglocktest", group = "Examples")
public class headinglocktest extends ActionOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private MotorControl motorControl;
    private MotorActions motorActions;
    private boolean specSensorTriggered = false;
    private PIDController headingPIDController;

    private DetectedColor allianceColor = DetectedColor.BLUE;
    private boolean autoOuttake = true;
    private boolean bangbangmode = false;

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
    private boolean spinActive = false;

    private boolean shouldHeadingLock = false;
    private boolean hasDetectedCorrectColor = false;
    private boolean headingLockDisabledByTrigger = false;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        headingPIDController = new PIDController(3, 0, 0.25);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        DetectedColor color = motorControl.getDetectedColor();
        boolean isCorrectSampleColor = (color == allianceColor || color == DetectedColor.YELLOW);
        boolean isLiftAtTargetHeight = motorControl.lift.closeEnough(100, 20);

        if (isCorrectSampleColor && !hasDetectedCorrectColor) {
            hasDetectedCorrectColor = true;
        }

        if (bangbangmode && hasDetectedCorrectColor && !headingLockDisabledByTrigger && isLiftAtTargetHeight) {
            shouldHeadingLock = true;
        } else {
            shouldHeadingLock = false;
        }

        double currentHeading = follower.getPose().getHeading();
        double rotation = 1;
        if (motorControl.extendo.motor.getCurrentPosition() > 50) {
            rotation = 0.5;
        }

        if (shouldHeadingLock) {
            double targetHeading = Math.toRadians(180);
            double headingError = targetHeading - currentHeading;
            headingError = Math.IEEEremainder(headingError, 2 * Math.PI);
            double headingCorrection = Math.abs(headingError) < Math.toRadians(2) ? 0 : headingPIDController.calculate(headingError);

            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    headingCorrection,
                    true
            );
        } else {
            follower.setTeleOpMovementVectors(
                    -gamepad1.right_stick_y,
                    -gamepad1.right_stick_x,
                    -gamepad1.left_stick_x * rotation,
                    true
            );
        }

        if (!ranInit){
            run(motorActions.safePositions());
            ranInit = true;
        }

        if (gamepad1.right_bumper && !rightBumperPressed) {
            if(bangbangmode){
                run(new SequentialAction(
                        new ParallelAction(
                                motorActions.specimenExtend(300),
                                motorActions.depositSpecimen()),
                        new SleepAction(0.3),
                        motorActions.intakeSpecimen()
                ));
            } else {
                run(motorActions.specimenExtend(300));
            }
            rightBumperPressed = true;
        } else if (!gamepad1.right_bumper) {
            rightBumperPressed = false;
        }

        if (gamepad1.left_bumper && !leftBumperPressed) {
            run(motorActions.spin.eat());
            hasDetectedCorrectColor = false;
            headingLockDisabledByTrigger = false;
            run(new SequentialAction(
                    new ParallelAction(
                            motorActions.grabUntilSpecimen(allianceColor)
                    ),
                    bangbangmode ? motorActions.extendo.waitUntilFinished() : motorActions.intakeTransfer(),
                    bangbangmode ? motorActions.spitSamplettele() : null
            ));
            leftBumperPressed = true;
        } else if (!gamepad1.left_bumper) {
            leftBumperPressed = false;
        }

        boolean correctColor = color == allianceColor || color == DetectedColor.YELLOW || motorControl.lift.getTargetPosition() > 70;

        if (correctColor) {
            if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
                run(motorActions.outtakeSpecimen());
                rightTriggerPressed = true;
            } else if (gamepad1.right_trigger == 0) {
                rightTriggerPressed = false;
            }

            if (gamepad1.left_trigger > 0 && !leftTriggerPressed) {
                hasDetectedCorrectColor = false;
                headingLockDisabledByTrigger = false;
                shouldHeadingLock = false;
                run(new SequentialAction(
                        motorActions.spin.poop(),
                        motorActions.spin.waitUntilEmpty(motorControl),
                        motorActions.lift.transfer()
                ));
                leftTriggerPressed = true;
            } else if (gamepad1.left_trigger == 0) {
                leftTriggerPressed = false;
            }
        } else {
            if (gamepad1.right_trigger > 0) {
                int target = Math.min(motorControl.extendo.motor.getCurrentPosition() + 100, 750);
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

        if (gamepad1.dpad_down && !dpadDownPressed) {
            if (!spinActive) {
                run(new SequentialAction(
                        motorActions.specimenExtend(650),
                        motorActions.spin.poop()
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

        if (gamepad1.dpad_up && !dpadUpPressed) {
            run(motorActions.outtakeSample());
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.square && !startPressed) {
            bangbangmode = !bangbangmode;
            startPressed = true;
        } else if (!gamepad1.square) {
            startPressed = false;
        }

        if (gamepad1.a) {
            run(motorActions.intakeSpecimen());
        } else if (gamepad1.b) {
            run(motorActions.outtakeSpecimen());
        } else if (gamepad1.y) {
            run(motorActions.specgone());
        }

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

        double specDist = motorControl.getSpecSensorDistanceCm();
        if (bangbangmode && specDist < 4 && !specSensorTriggered && motorControl.lift.closeEnough(0, 15)) {
            run(motorActions.outtakeSpecimen());
            specSensorTriggered = true;
        } else if (specDist >= 3.0) {
            specSensorTriggered = false;
        }

        follower.update();
        super.loop();
        motorControl.update();

        telemetry.addData("SpecSensorDist", specDist);
        telemetry.addData("SpecSensorTriggered", specSensorTriggered);
        telemetry.addData("HeadingLock", shouldHeadingLock);
        telemetry.addData("CorrectColor", isCorrectSampleColor);
        telemetry.addData("LiftAtHeight", isLiftAtTargetHeight);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("AutoOuttake", autoOuttake);
        telemetry.addData("bangbang", bangbangmode);
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
