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
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionHelpers.WaitUntilAction;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * SpecimenAuto is an autonomous OpMode that uses a series of PathChainTasks.
 * It extends PathChainAutoOpMode so that you only need to override buildPathChains() and buildTaskList(),
 * plus the dummy path-follower methods.
 */
@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;
    private Timer opModeTimer;  // additional timer if desired

    // -------- Poses --------
    private final Pose startPose = new Pose(9, 56, Math.toRadians(0));
    private final Pose preloadPose = new Pose(40, 70, Math.toRadians(0));
    private final Pose scorePose = new Pose(41, 68, Math.toRadians(25));
    private final Pose scorePose1 = new Pose(41, 68, Math.toRadians(25));
    private final Pose scorePose2 = new Pose(41, 68, Math.toRadians(25));
    private final Pose scorePose3 = new Pose(41, 68, Math.toRadians(25));
    private final Pose scorePose4 = new Pose(41, 68, Math.toRadians(25));
    private final Pose pickup1Pose = new Pose(20, 24, Math.toRadians(0));
    private final Pose pickup1Control = new Pose(22, 76, Math.toRadians(311));
    private final Pose pickup2Pose = new Pose(25, 35, Math.toRadians(315));
    private final Pose pickup3Pose = new Pose(28, 30, Math.toRadians(305));
    private final Pose depositPose1 = new Pose(25, 44, Math.toRadians(250));
    private final Pose depositPose2 = new Pose(25, 40, Math.toRadians(250));
    private final Pose intake = new Pose(11, 35, Math.toRadians(0));
    private final Pose intakeControl1 = new Pose(32, 35, Math.toRadians(0));
    private final Pose intakeControl2 = new Pose(5, 74, Math.toRadians(0));
    private final Pose intakeControl3 = new Pose(30, 34, Math.toRadians(0));
    private final Pose parkPose = new Pose(11, 22, Math.toRadians(90));
    private final Pose parkControlPose = new Pose(12, 74, Math.toRadians(90));

    private boolean spitDone1, spitDone2, spitDone3 = false;

    // --- Vision turn related fields ---
    private TurnTask visionTurn1, visionTurn2;
    private Vector2d latestVisionPose = new Vector2d(0, 0);
    private double latestVisionAngle = 0;

    private final AtomicBoolean specimenProcessingComplete = new AtomicBoolean(false);


    // -------- PathChains --------
    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup2, grabPickup3;
    private PathChain depositHP1, depositHP2, depositHP3;
    private PathChain intake1, intake2, intake3, intake4, intake5;
    private PathChain  score1, score2, score3, score4, score5;
    private PathChain  vision1deposit, vision2intake, vision2deposit;
    private PathChain  preoloadIntake;
    private PathChain parkChain;

    /**
     * Collects a single set of Limelight samples and stores the results
     * into {@code latestVisionPose} (horizontal, forward) and
     * {@code latestVisionAngle}. Returns true if successful.
     */
    private boolean collectVisionData() {
        limelight.startCollectingSamples();
        long start = System.currentTimeMillis();
        boolean success = false;
        while (opModeIsActive() && System.currentTimeMillis() - start < 1000) {
            if (limelight.collectSamples()) {
                Vector2d pose = limelight.getAveragePose();
                if (pose.x != 99.99) {
                    latestVisionPose = pose;
                    latestVisionAngle = limelight.getAverageAngle();
                    success = true;
                    break;
                }
            }
        }
        limelight.resetSamples();
        return success;
    }


    // -------- Override buildPathChains() --------
    @Override
    protected void buildPathChains() {
        // Intake path from deposit to intake.
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickup1Pose),
                        new Point(intakeControl3),
                        new Point(intake)))
                .setConstantHeadingInterpolation(Math.toRadians(intake.getHeading()))
                .addParametricCallback(0.5, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0, () -> run(motorActions.intakeSpecimen()))
                .build();



        score1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(intake),
                        new Point(scorePose1)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0, () -> motorControl.spin.setPower(0))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(intake),
                        new Point(scorePose2)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(intake),
                        new Point(scorePose3)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(intake),
                        new Point(scorePose4)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        // Intake path from score to intake.
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intake)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),
                        intake.getHeading(), 50)
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0.2, () -> run(motorActions.intakeSpecimen()))
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intake)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),
                        intake.getHeading(), 50)
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0.2, () -> run(motorActions.intakeSpecimen()))
                .build();

        intake4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intake)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),
                        intake.getHeading(), 50)
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0.2, () -> run(motorActions.intakeSpecimen()))
                .build();

        intake5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intake)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),
                        intake.getHeading(), 50)
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0.2, () -> run(motorActions.intakeSpecimen()))
                .build();

        score5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(intake),
                        new Point(scorePose4)))
                .setConstantHeadingInterpolation(scorePose.getHeading())

                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();


        // Preload path.
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0, () ->  run(motorActions.safePositions()))
                .build();


        // Park path.
        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose4),
                        new Point(parkControlPose),
                        new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();

        vision1deposit = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(preloadPose),
                        new Point(intake)))
                .addParametricCallback(0.2, () -> run(new SequentialAction(
                                motorActions.spitSample(),
                                motorActions.spin.waitUntilEmpty(motorControl),
                        motorActions.lift.transfer())))
                .addParametricCallback(0.85, () -> run(motorActions.spin.poop()))
                .setLinearHeadingInterpolation(preloadPose.getHeading(),intake.getHeading())
                .build();

        vision2intake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(scorePose)))
                .addParametricCallback(0, ()->run(motorActions.spin.stop()))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        vision2deposit = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(pickup1Pose)))
                .addParametricCallback(0, ()->run(motorActions.spin.stop()))
                .addParametricCallback(0, () -> run(motorActions.spitSample()))
                .addParametricCallback(0.85, () -> run(motorActions.spin.poop()))
                .addParametricCallback(1, () -> run(motorActions.extendo.extended()))
                .setConstantHeadingInterpolation(
                        Math.toRadians(pickup1Pose.getHeading()))
                .build();


    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Preload task.
        addPath(scorePreload, 1).addWaitAction(0,
                new SequentialAction(
                        motorActions.depositSpecimen(),
                        motorActions.lift.vision()
                ));

        // Vision-based turn before first vision deposit
        visionTurn1 = addRelativeTurnDegrees(0, true, 0);

        addPath(vision1deposit, 0.1).addWaitAction(0,motorActions.outtakeSpecimen());



        addPath(vision2intake, 1).addWaitAction(0,
                new SequentialAction(
                        motorActions.depositSpecimen(),
                        motorActions.lift.vision()
                ));

        // Vision-based turn before second vision deposit
        visionTurn2 = addRelativeTurnDegrees(0, true, 0);

        addPath(vision2deposit, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.set(640),
                        new SleepAction(0.05),
                        motorActions.grabUntilSpecimen(),
                        motorActions.spitSample(),
                        motorActions.extendo.retracted(),
                        motorActions.extendo.waitUntilFinished(),
                        telemetryPacket -> {
                            spitDone1 = true; return false;
                        },
                        motorActions.extendo.findZero(),
                        motorActions.spin.poop(),
                        motorActions.spin.waitUntilEmpty(motorControl),
                        telemetryPacket -> {
                            spitDone1 = true; return false;
                        }
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone1)
        ;

        addTurnToDegrees(-15, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.spin.waitUntilEmpty(motorControl),
                        motorActions.extendo.set(660),
                        new SleepAction(0.05),
                        motorActions.grabUntilSpecimen(),
                        motorActions.spitSample(),
                        motorActions.extendo.retracted(),
                        motorActions.extendo.waitUntilFinished(),
                        telemetryPacket -> {
                            spitDone2 = true; return false;
                        },
                        motorActions.extendo.findZero(),
                        motorActions.spin.poop(),
                        motorActions.spin.waitUntilEmpty(motorControl)
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone2)
        ;


        addTurnToDegrees(-34, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.spin.waitUntilEmpty(motorControl),
                        motorActions.extendo.set(880),
                        new SleepAction(0.05),
                        motorActions.grabUntilSpecimen(),
                        motorActions.spitSample(),
                        motorActions.extendo.retracted(),
                        motorActions.extendo.waitUntilFinished(300),
                        telemetryPacket -> {
                            spitDone3 = true; return false;
                        },
                        motorActions.extendo.waitUntilFinished(),
                        motorActions.extendo.findZero(),
                        motorActions.spin.poop(),
                        motorActions.spin.waitUntilEmpty(motorControl),
                        motorActions.intakeSpecimen()
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone3)
        ;

        addPath(intake1, 0.1);


        // Score task 1.
        tasks.add(new PathChainTask(score1, 0));

        // Intake task 2.
        tasks.add(new PathChainTask(intake2, 0.2));

        // Score task 2.
        tasks.add(new PathChainTask(score2, 0));

        // Intake task 3.
        tasks.add(new PathChainTask(intake3, 0.2));
        // Score task 3.
        tasks.add(new PathChainTask(score3, 0));

        // Intake task 4.
        tasks.add(new PathChainTask(intake4, 0.2));

        // Score task 4.
        tasks.add(new PathChainTask(score4, 0));

        tasks.add(new PathChainTask(intake5, 0.2));

        tasks.add(new PathChainTask(score5, 0));



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





        // Build the paths and tasks.
        buildPathChains();
        buildTaskList();
    }

    @Override
    protected void startTurn(TurnTask task) {
        if (task == visionTurn1 || task == visionTurn2) {
            if (collectVisionData()) {
                double inches = Math.hypot(latestVisionPose.x, latestVisionPose.y);
                motorControl.extendo.setTargetPosition(inches * 32.0);
                task.angle = Math.abs(latestVisionAngle);
                task.isLeft = latestVisionAngle > 0;
                task.useDegrees = true;
                task.isRelative = true;
            } else {
                task.angle = 0;
                task.isLeft = true;
                task.useDegrees = true;
                task.isRelative = true;
            }
        }

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
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("isEmpty",motorControl.isEmpty());
        telemetry.addData("extendoReset",motorControl.extendo.resetting);
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.update();
    }
}
