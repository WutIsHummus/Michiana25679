package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.data.AngleUtils;
import org.firstinspires.ftc.teamcode.helpers.data.Enums.DetectedColor;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Spec Tele Blue", group = "Examples")
public class specimenteleop extends ActionOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private MotorControl motorControl;
    private MotorActions motorActions;
    private boolean specSensorTriggered = false;
    private boolean hangMode = false;

    private boolean gapIntakePressed = false;

    private DetectedColor allianceColor = DetectedColor.BLUE;
    private boolean autoOuttake = true;
    public boolean optionpressed = false;
    boolean right2BumperPressed = false;

    private boolean bangbangmode = false;

    // Edge-detect flags
    private boolean startPressed = false;
    private boolean xPressed = false;
    private boolean rightBumperPressed = false;

    private boolean joystickDown = false;
    private boolean joystickUp = false;
    private boolean leftBumperPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftTriggerPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadUpPressed = false;
    private boolean squarePressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean headingLock = false;

    private boolean liftReseted = false;
    private boolean ranInit = false;

    private PIDController headingController;
    private double targetHeading = 0;

    // Track spinning
    private boolean spinActive = false;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);

        headingController = new PIDController(0.05, 0.0, 0.0005);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        if (!ranInit){
            run(motorActions.safePositions());
            run(motorActions.hang.hang());
            run(motorActions.lift.findZero());
            ranInit = true;
        }


        // Toggle alliance color
        if (gamepad2.x && !xPressed) {
            allianceColor = (allianceColor == DetectedColor.RED)
                    ? DetectedColor.BLUE
                    : DetectedColor.RED;
            xPressed = true;
        } else if (!gamepad2.x) {
            xPressed = false;
        }

        // RIGHT BUMPER: always extend sample
        if (gamepad1.right_bumper && !rightBumperPressed) {
            if(bangbangmode){
            run(
                    new SequentialAction(
                            new ParallelAction(
                                    motorActions.specimenExtend(300),
                                    motorActions.depositSpecimen()),
                            new SleepAction(0.3),
                            motorActions.intakeSpecimen()

                    ));
            rightBumperPressed = true;}
            else {
                run(motorActions.specimenExtend(300));
            }
        } else if (!gamepad1.right_bumper) {
            rightBumperPressed = false;
        }


        if (gamepad2.right_bumper && ! right2BumperPressed) {
            run(motorActions.specimenspittele(motorControl.extendo.getTargetPosition()));
            right2BumperPressed = true;
        } else if (!gamepad2.right_bumper) {
            right2BumperPressed = false;
        }



        // LEFT BUMPER: grab & optional auto-outtake
        if (gamepad1.left_bumper && !leftBumperPressed) {
            run(motorActions.spin.eat());
            if (bangbangmode) {
                run(new SequentialAction(
                        new ParallelAction(
                                //motorActions.outtakeTransfer(),
                                motorActions.grabUntilSpecimen(allianceColor)
                        ),
                        motorActions.extendo.waitUntilFinished(),
                        motorActions.spitSamplettele2()
                ));
            } else {
                run(new SequentialAction(
                        new ParallelAction(
                                //motorActions.outtakeTransfer(),
                                motorActions.grabUntilSpecimen(allianceColor)
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
                || motorControl.lift.getTargetPosition() > 70;

        if (correctColor) {
            // RIGHT TRIGGER: outtake sample
            if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
                run(motorActions.outtakeSpecimen());
                rightTriggerPressed = true;
            } else if (gamepad1.right_trigger == 0) {
                rightTriggerPressed = false;
            }
            // LEFT TRIGGER: outtake transfer
            if (gamepad1.left_trigger > 0 && !leftTriggerPressed) {
                run(new SequentialAction(motorActions.spin.poop(),
                                motorActions.spin.waitUntilEmpty(motorControl),
                                motorActions.lift.transfer(), new SleepAction(0.2))
                        );

                leftTriggerPressed = true;
            } else if (gamepad1.left_trigger == 0) {
                leftTriggerPressed = false;
            }
        } else {
            // WRONG COLOR: move extendo
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

        // D-pad down: toggle spin vs intake
        if (gamepad1.dpad_down && !dpadDownPressed) {
            if (!spinActive) {
                run(new SequentialAction(
                        motorActions.specimenExtendtele(650)
                ));
                spinActive = true;
            } else {
                run(new ParallelAction(
                        motorActions.spin.stop(),
                        motorActions.intakeTransfer2()
                ));
                spinActive = false;
            }
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // D-pad up: outtake sample
        if (gamepad1.dpad_up && !dpadUpPressed) {
            if (motorControl.lift.motor.getCurrentPosition() < 50) {
                run(new SequentialAction(
                        motorActions.linkage.transfer(),
                        new SleepAction(0.2),
                        motorActions.claw.partclosetransfer(),
                        new SleepAction(0.2),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        motorActions.spin.slowpoop(),
                                        motorActions.outArm.middle(),
                                        motorActions.linkage.transfer()
                                ),
                                motorActions.lift.sample(),
                                motorActions.inArm.specimenExtended(),
                                motorActions.inPivot.specimenExtended()
                        ),
                        motorActions.linkage.transfer(),
                        motorActions.lift.waitUntilFinished(800),
                        motorActions.outArm.sampleScore()
                ));
            } else {
                run(motorActions.outtakeTransfer()); // fallback to old behavior if lift isn't low
            }
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }


        if (gamepad1.share && !gapIntakePressed) {
            run(new SequentialAction(
                    motorActions.outArm.sampleScoreAuto(),
                    new SleepAction(0.2),
                    motorActions.linkage.transfer(),
                    new SleepAction(0.2),
                    motorActions.outArm.specimenIntake()
            ));
            gapIntakePressed = true;
        } else if (!gamepad1.share) {
            gapIntakePressed = false;
        }
        if (gamepad1.square && !squarePressed) {
            bangbangmode = !bangbangmode;
            squarePressed = true;
        } else if (!gamepad1.square) {
            squarePressed = false;
        }

        if (gamepad1.options && !optionpressed) {
            run(motorActions.intakeTransfer5());
            optionpressed = true;
        } else if (!gamepad1.options) {
            optionpressed = false;
        }

        // Specimen controls
        if (gamepad1.a) {
            run(motorActions.intakeSpecimen());
        } else if (gamepad1.b) {
            run(motorActions.outtakeSpecimen());
        } else if (gamepad1.y) {
            run(motorActions.specgone());
        }



        // Hang controls
        if (gamepad1.dpad_left && !dpadLeftPressed) {
            run(motorActions.hang.up());
            run(motorActions.lift.set(820));
            dpadLeftPressed = true;
        } else if (!gamepad1.dpad_left) {
            hangMode = true;
            dpadLeftPressed = false;
        }

        if (gamepad1.dpad_right && !dpadRightPressed) {
            run( new SequentialAction(
                    motorActions.hang.hang(),
                    motorActions.ptolLatch.lock(),
                    motorActions.ptorLatch.lock()
                    //motorActions.outArm.specimenIntake()
            ));

            dpadRightPressed = true;

        }
        else if (dpadRightPressed && gamepad1.right_stick_y > 0){
            run(new SequentialAction(
                    motorActions.lift.nothing(true),
                    new SleepAction(0.5),
                    motorActions.hang.down()
            ));
        }

        double specDist = motorControl.getSpecSensorDistanceCm();

        if (bangbangmode && specDist < 4 && !specSensorTriggered && motorControl.lift.closeEnough(0,15)) {
            run(motorActions.outtakeSpecimen2());
            specSensorTriggered = true;  // mark as triggered
        } else if (specDist >= 3.0) {
            specSensorTriggered = false; // reset when sensor clears
        }




        double rotation = 1;
        if (motorControl.extendo.motor.getCurrentPosition() > 50) {
            rotation = 0.5;
        }

        if (gamepad2.a && !startPressed) {
            headingLock = !headingLock;
            if (headingLock) {
                targetHeading = AngleUtils.normalizeRadians(follower.getPose().getHeading());
                headingController.setSetPoint(targetHeading);
                headingController.reset();
            }
            startPressed = true;
        } else if (!gamepad2.a) {
            startPressed = false;
        }

        double drive  = -gamepad1.right_stick_y;
        double strafe = -gamepad1.right_stick_x;
        double turn;
        if (headingLock) {
            double current = AngleUtils.normalizeRadians(follower.getPose().getHeading());
            turn = headingController.calculate(current);
        } else {
            turn = -gamepad1.left_stick_x * rotation;
        }

        if (gamepad2.right_stick_y> 0 && !joystickUp){
            joystickUp = true;
            run(motorActions.lift.set(motorControl.lift.getTargetPosition() + 50));
        }
        else {
            joystickUp = false;
        }

        if (gamepad2.right_stick_y < 0&& !joystickDown){
            joystickDown = true;
            run(motorActions.lift.set(motorControl.lift.getTargetPosition() - 50));

        }
        else {
            joystickDown = false;
        }

        if (gamepad2.b && !liftReseted){
            liftReseted = true;
            run(motorActions.lift.findZero());

        }
        else {
            liftReseted = false;
        }



        follower.setTeleOpMovementVectors(drive, strafe, turn);



        follower.update();
        super.loop();
        motorControl.update();


        // Telemetry
        telemetry.addData("SpecSensorDist", specDist);

        telemetry.addData("SpecSensorTriggered", specSensorTriggered);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("AutoOuttake", autoOuttake);
        telemetry.addData("bangbang", bangbangmode);
        telemetry.addData("headinglock", headingLock);
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
