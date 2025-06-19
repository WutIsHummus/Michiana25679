package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.data.AngleUtils;
import org.firstinspires.ftc.teamcode.helpers.data.Enums.DetectedColor;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Spec Tele", group = "Examples")
public class specimenteleop extends ActionOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private MotorControl motorControl;
    private MotorActions motorActions;
    private boolean specSensorTriggered = false;


    private DetectedColor allianceColor = DetectedColor.BLUE;
    private boolean autoOuttake = true;

    private boolean bangbangmode = false;

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
    private boolean headingLock = false;
    private boolean ranInit = false;

    private PIDFController headingController;
    private double targetHeading = 0;

    // Track spinning
    private boolean spinActive = false;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);

        headingController = new PIDFController(new CustomPIDFCoefficients(2.6, 0, 0, 0));
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
                        motorActions.spitSamplettele()
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
                || color == DetectedColor.YELLOW
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
                                motorActions.lift.transfer())
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

        // D-pad up: outtake sample
        if (gamepad1.dpad_up && !dpadUpPressed) {
            run(motorActions.outtakeSample());
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }


        if (gamepad1.square && !squarePressed) {
            bangbangmode = !bangbangmode;
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
            run(motorActions.specgone());
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

        double specDist = motorControl.getSpecSensorDistanceCm();

        if (bangbangmode && specDist < 4 && !specSensorTriggered && motorControl.lift.closeEnough(0,15)) {
            run(motorActions.outtakeSpecimen());
            specSensorTriggered = true;  // mark as triggered
        } else if (specDist >= 3.0) {
            specSensorTriggered = false; // reset when sensor clears
        }




        if (gamepad2.a && !startPressed) {
            headingLock = !headingLock;
            if (headingLock) {
                // latch the setpoint
                targetHeading = AngleUtils.normalizeRadians(follower.getPose().getHeading());
                headingController.reset();   // clear I/D state
            }
            startPressed = true;
        } else if (!gamepad2.a) {
            startPressed = false;
        }

        double drive  = -gamepad1.right_stick_y;
        double strafe = -gamepad1.right_stick_x;
        double rotation = motorControl.extendo.motor.getCurrentPosition() > 50 ? 0.5 : 1.0;
        double turn;

        if (headingLock) {
            double current = AngleUtils.normalizeRadians(follower.getPose().getHeading());
            double error = AngleUtils.shortestAngleDifference(current, targetHeading);
            headingController.updateError(error);
            turn = headingController.runPIDF();
        } else {
            turn = -gamepad1.left_stick_x * rotation;
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
