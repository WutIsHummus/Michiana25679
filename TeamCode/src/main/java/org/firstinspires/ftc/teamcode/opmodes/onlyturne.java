package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.helpers.data.AngleUtils;
import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * SpecimenAuto is an autonomous OpMode that uses a series of PathChainTasks.
 * It extends PathChainAutoOpMode so that you only need to override buildPathChains() and buildTaskList(),
 * plus the dummy path-follower methods.
 */
@Autonomous(name = "drivetothird")
public class onlyturne extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private Timer opModeTimer;  // additional timer if desired

    // -------- Poses --------
    private final Pose startPose   = new Pose(9, 111, Math.toRadians(270));
    private final Pose scorePose   = new Pose(18, 130, Math.toRadians(340));

    private final Pose subscorepose   = new Pose(18, 126, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(20, 124, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(20, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(24, 128, Math.toRadians(23));

    // Park poses
    private final Pose parkPose        = new Pose(62, 95, Math.toRadians(270));

    private final Pose subintakepose        = new Pose(62, 97, Math.toRadians(270));
    private final Pose parkControlPose = new Pose(70, 120, Math.toRadians(270));
    private MotorControl.Limelight limelight;

    private boolean scan1Done, scan2Done = false;
    private boolean scan3Done = false;
    private boolean scan4Done = false;


    private boolean eatdone1, eatdone2, eatdone3, firstcycle, depodone1, depodone2 = false;

    // --- Vision turn related fields ---
    private TurnTask visionTurn1, visionTurn2, visionTurn3, visionTurn4;

    private Vector2d latestVisionPose = new Vector2d(0, 0);
    private double latestVisionAngle = 0;
    private double lastDistance = 0;

    private boolean subscoreDone1 = false;
    private boolean subscoreDone2 = false;
    private boolean subscoreDone3 = false;
    private boolean subscoreDone4 = false;


    private final AtomicBoolean specimenProcessingComplete = new AtomicBoolean(false);


    // -------- PathChains --------
    private PathChain scorePreload;
    private PathChain intake1, intake2, intake3;
    private PathChain score1, score2, score3;

    private PathChain subscore1, subscore2, subscore3, subscore4;
    private PathChain parkChain1, parkChain2, parkChain3, parkChain4;


    private boolean visionStarted     = false;
    private double  visionStartTime   = 0.0;  // in seconds

    /**
     * Collects a single set of Limelight samples and stores the results
     * into {@code latestVisionPose} (horizontal, forward) and
     * {@code latestVisionAngle}. Returns true if successful.
     */


    // -------- Override buildPathChains() --------
    @Override
    protected void buildPathChains() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeSampleAuto()))
                .build();

        // ============ Park path ============
        // ============ Four Park Chains ============
        parkChain1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .addPath(new BezierCurve(
                        new Point(parkPose),
                        new Point(subintakepose)
                ))                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeTransfer()))
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.inArm.sampleExtended(),
                        motorActions.inPivot.sampleExtended()
                )))
                .addParametricCallback(0.5, () -> run(motorActions.sweeper.extended()))
                .addParametricCallback(1, () -> run(motorActions.sweeper.retracted()))
                .build();

        parkChain2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .addPath(new BezierCurve(
                        new Point(parkPose),
                        new Point(subintakepose)
                ))                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeTransfer()))
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.inArm.sampleExtended(),
                        motorActions.inPivot.sampleExtended()
                )))
                .addParametricCallback(0.5, () -> run(motorActions.sweeper.extended()))
                .addParametricCallback(1, () -> run(motorActions.sweeper.retracted()))
                .build();

        parkChain3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .addPath(new BezierCurve(
                        new Point(parkPose),
                        new Point(subintakepose)
                ))                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeTransfer()))
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.inArm.sampleExtended(),
                        motorActions.inPivot.sampleExtended()
                )))
                .addParametricCallback(0.5, () -> run(motorActions.sweeper.extended()))
                .addParametricCallback(1, () -> run(motorActions.sweeper.retracted()))
                .build();

        parkChain4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .addPath(new BezierCurve(
                        new Point(parkPose),
                        new Point(subintakepose)
                ))                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeTransfer()))
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.inArm.sampleExtended(),
                        motorActions.inPivot.sampleExtended()
                )))
                .addParametricCallback(0.5, () -> run(motorActions.sweeper.extended()))
                .addParametricCallback(1, () -> run(motorActions.sweeper.retracted()))
                .build();


        subscore1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.intakeTransfer(),
                        motorActions.outtakeSampleAuto2()
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
                .build();

        subscore2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.intakeTransfer(),
                        motorActions.outtakeSampleAuto2()
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
                .build();

        subscore3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.intakeTransfer(),
                        motorActions.outtakeSampleAuto2()
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
                .build();

        subscore4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.intakeTransfer(),
                        motorActions.outtakeSampleAuto2()
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading(), 50)
                .addParametricCallback(0, () -> run(new ParallelAction(motorActions.inArm.specimenGrab(),
                        motorActions.inPivot.specimenGrab(),
                        motorActions.spin.eat())))
                .addParametricCallback(0, () -> run(motorActions.extendo.set(400)))
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading(), 50)
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(motorActions.outtakeSampleAuto()
                ))
                .build();

    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        PathChainTask preloadTask = new PathChainTask(scorePreload, 0.05)
                .setMaxWaitTime(8)
                .addWaitAction(0, new SequentialAction(
                                motorActions.lift.waitUntilFinished(),
                                motorActions.cycle(),
                                telemetryPacket -> {
                                    firstcycle = true; return false;
                                }

                        )

                )
                .setWaitCondition(() -> firstcycle);

        tasks.add(preloadTask);

        addTurnToDegrees(350, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.set(600),
                        new SleepAction(0.05),
                        motorActions.inArm.specimenGrab(),
                        motorActions.inPivot.specimenGrab(),
                        motorActions.spin.eat(),
                        motorActions.extendo.waitUntilFinished(),
                        telemetryPacket -> {
                            eatdone1 = true; return false;
                        },
                        motorActions.intakeTransfer(),
                        motorActions.extendo.waitUntilFinished(),
                        motorActions.extendo.findZero()
                ))
                .setMaxWaitTime(3)
                .setWaitCondition(() -> eatdone1)
        ;

        addTurnToDegrees(315, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeSampleAuto(),
                        motorActions.extendo.set(400),
                        telemetryPacket -> {
                            depodone1 = true; return false;
                        }

                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> depodone1)
        ;

        PathChainTask pickup3Task = new PathChainTask(intake3, 0.2)
                .setMaxWaitTime(1.25)
                .setWaitCondition(() -> motorControl.extendo.closeEnough());
        tasks.add(pickup3Task);

        PathChainTask score3Task = new PathChainTask(score3, 0.4)
                .setWaitCondition(() -> motorControl.lift.closeEnough(800));
        tasks.add(score3Task);

//        addTurnToDegrees(15, 0)
//                .addWaitAction(0, new SequentialAction(
//                        motorActions.extendo.set(660),
//                        new SleepAction(0.05),
//                        motorActions.inArm.specimenGrab(),
//                        motorActions.inPivot.specimenGrab(),
//                        motorActions.spin.eat(),
//                        motorActions.extendo.waitUntilFinished(),
//                        telemetryPacket -> {
//                            eatdone2 = true; return false;
//                        },
//                        motorActions.intakeTransfer(),
//                        motorActions.extendo.waitUntilFinished(),
//                        motorActions.extendo.findZero()
//                ))
//                .setMaxWaitTime(1.5)
//                .setWaitCondition(() -> eatdone2)
//        ;
//
//        addTurnToDegrees(315, 0)
//                .addWaitAction(0, new SequentialAction(
//                        motorActions.outtakeSampleAuto(),
//                        motorActions.lift.waitUntilFinished(),
//                        new SleepAction(0.5),
//                        telemetryPacket -> {
//                            depodone2 = true; return false;
//                        }
//
//                ))
//                .setMaxWaitTime(2)
//                .setWaitCondition(() -> depodone2)
//        ;

        // Vision + parkChain1
        addPath(parkChain1, 0)
                .addWaitAction(0, new SequentialAction(
                        telemetryPacket -> {
                            MotorControl.Limelight.DetectionResult dr = limelight.getDistance();
                            if (dr != null) {
                                latestVisionAngle = Math.min(15, Math.max(-dr.yawDegrees, -15));
                                double yawRad = Math.toRadians(dr.yawDegrees);
                                double direct = dr.distanceInches / Math.cos(yawRad);
                                latestVisionPose = new Vector2d(direct * Math.sin(yawRad), direct * Math.cos(yawRad));
                            }
                            scan1Done = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(6)
                .setWaitCondition(() -> scan1Done);
        visionTurn1 = addRelativeTurnDegrees(0, true, 3)
                .addWaitAction(2.5, motorActions.intakeTransfer());

        addPath(subscore1, 0.2)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeSpecimen(),
                        telemetryPacket -> {
                            subscoreDone1 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(3)
                .setWaitCondition(() -> subscoreDone1);

// Vision + parkChain2
        addPath(parkChain2, 0)
                .addWaitAction(0, new SequentialAction(
                        telemetryPacket -> {
                            MotorControl.Limelight.DetectionResult dr = limelight.getDistance();
                            if (dr != null) {
                                latestVisionAngle = Math.min(15, Math.max(-dr.yawDegrees, -15));
                                double yawRad = Math.toRadians(dr.yawDegrees);
                                double direct = dr.distanceInches / Math.cos(yawRad);
                                latestVisionPose = new Vector2d(direct * Math.sin(yawRad), direct * Math.cos(yawRad));
                            }
                            scan2Done = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(6)
                .setWaitCondition(() -> scan2Done);
        visionTurn2 = addRelativeTurnDegrees(0, true, 3)
                .addWaitAction(2.5, motorActions.intakeTransfer());

        addPath(subscore2, 0.2)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeSpecimen(),
                        telemetryPacket -> {
                            subscoreDone2 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(3)
                .setWaitCondition(() -> subscoreDone2);

// Vision + parkChain3
        addPath(parkChain3, 0)
                .addWaitAction(0, new SequentialAction(
                        telemetryPacket -> {
                            MotorControl.Limelight.DetectionResult dr = limelight.getDistance();
                            if (dr != null) {
                                latestVisionAngle = Math.min(15, Math.max(-dr.yawDegrees, -15));
                                double yawRad = Math.toRadians(dr.yawDegrees);
                                double direct = dr.distanceInches / Math.cos(yawRad);
                                latestVisionPose = new Vector2d(direct * Math.sin(yawRad), direct * Math.cos(yawRad));
                            }
                            scan3Done = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(6)
                .setWaitCondition(() -> scan3Done);
        visionTurn3 = addRelativeTurnDegrees(0, true, 3)
                .addWaitAction(2.5, motorActions.intakeTransfer());

        addPath(subscore3, 0.2)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeSpecimen(),
                        telemetryPacket -> {
                            subscoreDone3 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(3)
                .setWaitCondition(() -> subscoreDone3);

// Vision + parkChain4
        addPath(parkChain4, 0)
                .addWaitAction(0, new SequentialAction(
                        telemetryPacket -> {
                            MotorControl.Limelight.DetectionResult dr = limelight.getDistance();
                            if (dr != null) {
                                latestVisionAngle = Math.min(15, Math.max(-dr.yawDegrees, -15));
                                double yawRad = Math.toRadians(dr.yawDegrees);
                                double direct = dr.distanceInches / Math.cos(yawRad);
                                latestVisionPose = new Vector2d(direct * Math.sin(yawRad), direct * Math.cos(yawRad));
                            }
                            scan4Done = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(6)
                .setWaitCondition(() -> scan4Done);
        visionTurn4 = addRelativeTurnDegrees(0, true, 3)
                .addWaitAction(2.5, motorActions.intakeTransfer());

        addPath(subscore4, 0.2)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeSpecimen(),
                        telemetryPacket -> {
                            subscoreDone4 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(3)
                .setWaitCondition(() -> subscoreDone4);


    }


    // -------- Override dummy follower methods --------
    @Override
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected boolean isTurning() {

        if (!isActivelyTurningInternalFlag) {
            return false;
        }

        // This is just an example, replace with your actual follower's heading retrieval
        double currentHeading = follower.getPose().getHeading();

        // Calculate the shortest difference, handling wrap-around
        double headingError = AngleUtils.shortestAngleDifference(targetHeadingRadians ,currentHeading);

        if (Math.abs(headingError) < HEADING_TOLERANCE) {
            isActivelyTurningInternalFlag = false;
            follower.breakFollowing();
            return false;
        }

        if (!follower.isBusy()) {
            isActivelyTurningInternalFlag = false;
            return false;
        }

        return true;
    }

    @Override
    protected double getCurrentTValue() {
        return follower.getCurrentTValue();
    }

    @Override
    protected void startPath(PathChainTask task) {
        isActivelyTurningInternalFlag = false;
        follower.followPath((PathChain) task.pathChain, true);
    }

    // -------- Standard OpMode Lifecycle Methods --------
    @Override
    public void init() {
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);




        limelight     = new MotorControl.Limelight(hardwareMap, telemetry);

        limelight.setPrimaryClass("red");

        // Build the paths and tasks.
        buildPathChains();
        buildTaskList();
    }

    @Override
    protected void startTurn(TurnTask task) {
        if (task == visionTurn1 || task == visionTurn2) {


            double inches = Math.hypot(latestVisionPose.x, latestVisionPose.y);
            lastDistance = inches;

            if (lastDistance==0) return;

            if (lastDistance != 0) {
                task.addWaitAction(0, new SequentialAction(
                        motorActions.sampleExtend(Math.min(inches * 32.25, 800)),
                        motorActions.extendo.waitUntilFinished(),
                        motorActions.extendo.set(Math.min((inches + 3) * 32.25, 800)),
                        motorActions.spin.eat(),
                        motorActions.spin.eatUntilStrict(Enums.DetectedColor.RED, motorControl)
                ));
            }

            // Now set the turn angle/side based on the stored vision angle:
            task.angle      = Math.abs(latestVisionAngle);
            task.isLeft     = (latestVisionAngle > 0);
            task.useDegrees = true;
            task.isRelative = true;

        }
        task.initiated = true;
        Pose currentRobotPose = follower.getPose();
        double currentX = currentRobotPose.getX();
        double currentY = currentRobotPose.getY();
        double currentHeadingRadians = currentRobotPose.getHeading();

        if (task.isRelative) {
            double turnAmountRadians = task.useDegrees ? Math.toRadians(task.angle) : task.angle;
            targetHeadingRadians = currentHeadingRadians + (task.isLeft ? turnAmountRadians : -turnAmountRadians);
        } else {
            targetHeadingRadians = task.useDegrees ? Math.toRadians(task.angle) : task.angle;
        }

        targetHeadingRadians = AngleUtils.normalizeRadians(targetHeadingRadians);

        PathChain turnPath = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(new Pose(currentX, currentY, currentHeadingRadians))))
                .setConstantHeadingInterpolation(targetHeadingRadians)
                .build();

        follower.followPath(turnPath, true);

        isActivelyTurningInternalFlag = true;
    }


    @Override
    public void start() {
        opModeTimer.resetTimer();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        runTasks();
        motorControl.update();


        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Turning", isTurning());
        telemetry.addData("PathActive", isPathActive());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("liftCurrent",motorControl.lift.motor.getCurrentPosition());
        telemetry.addData("liftTarget",motorControl.lift.getTargetPosition());
        telemetry.addData("isEmpty",motorControl.isEmpty());
        telemetry.addData("extendoReset",motorControl.extendo.resetting);
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.addData("Angle", latestVisionAngle);
        telemetry.addData("Pose", latestVisionPose);
        telemetry.addData("Distance", lastDistance);
        telemetry.update();
    }
}
