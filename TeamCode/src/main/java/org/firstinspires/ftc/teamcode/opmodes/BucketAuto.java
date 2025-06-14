package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Bucket Sample")
public class BucketAuto extends PathChainAutoOpMode {

    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;

    private final Pose startPose   = new Pose(9, 111, Math.toRadians(270));
    private final Pose scorePose   = new Pose(17, 128, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(20, 122, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(20, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(24, 128, Math.toRadians(20));
    private final Pose parkPose        = new Pose(62, 97, Math.toRadians(315));
    private final Pose parkControlPose = new Pose(64.5, 116, Math.toRadians(270));

    private PathChain scorePreload;
    private PathChain intake1, intake2, intake3;
    private PathChain score1, score2, score3;
    private PathChain parkChain;


    @Override
    protected void startTurn(TurnTask task) {
        if (task.isRelative) {
            // Relative turn
            if (task.useDegrees) {
                follower.turnDegrees(task.angle, task.isLeft);
            } else {
                follower.turn(task.angle, task.isLeft);
            }
        } else {
            // Absolute turn
            if (task.useDegrees) {
                follower.turnToDegrees(task.angle);
            } else {
                follower.turnTo(task.angle);
            }
        }
    }

    @Override
    protected boolean isTurning() {
        return follower.isTurning();
    };

    @Override
    protected void buildPathChains() {
        // ============ Intake paths ============
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .addParametricCallback(0.6, () -> run(motorActions.grabUntilSample(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1, () -> run(motorActions.extendo.set(250)))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .addParametricCallback(0.6, () -> run(motorActions.grabUntilSample(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1, () -> run(motorActions.extendo.set(350)))
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading(), 50)
                .addParametricCallback(0.6, () -> run(motorActions.grabUntilSample(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1, () -> run(motorActions.extendo.set(400)))
                .build();

        // ============ Score paths ============
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), 50)
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(10),
                        motorActions.outtakeSample()
                )))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading(), 50)
                .addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(10),
                        motorActions.outtakeSample()
                )))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading(), 50)
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(10),
                        motorActions.outtakeSample()
                )))
                .build();

        // ============ Preload path ============
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePose)))
                .addParametricCallback(0, () -> run(motorActions.outtakeSample()))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // ============ Park path ============
        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.inArm.sampleExtended(),
                        motorActions.inPivot.sampleExtended()
                )))
                .build();
    }


    @Override
    protected void buildTaskList() {
        tasks.clear();

        PathChainTask preloadTask = new PathChainTask(scorePreload, 0.5)
                /*
                .addWaitAction(
                        () -> motorControl.lift.closeEnough(770)

                    new SequentialAction(
                        motorActions.outtakeLinkage.Retract(),
                        new SleepAction(0.4),
                        motorActions.outtakeClaw.Open(),
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Transfer()
                    )

                )
                .setMaxWaitTime(2) */
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(preloadTask);

        PathChainTask pickup1Task = new PathChainTask(intake1, 0.5)
                .setMaxWaitTime(1.25)
                /*
                .addWaitAction(0.5, motorActions.extendo.setTargetPosition(500))
                .addWaitAction(1,   motorActions.intakeGrabUntil(Enums.DetectedColor.UNKNOWN))
                */
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup1Task);

        PathChainTask score1Task = new PathChainTask(score1, 0.5)
                /*
                .addWaitAction(
                    () -> motorControl.lift.closeEnough(770),
                    new SequentialAction(
                        motorActions.outtakeLinkage.Retract(),
                        new SleepAction(0.4),
                        motorActions.outtakeClaw.Open(),
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Transfer()
                    )
                )
                */
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(score1Task);

        PathChainTask pickup2Task = new PathChainTask(intake2, 0.5)
                .setMaxWaitTime(1.25)
                /*
                .addWaitAction(
                    0,
                    () -> motorControl.getDetectedColor() == Enums.DetectedColor.UNKNOWN,
                    motorActions.extendo.setTargetPosition(500)
                )
                .addWaitAction(1, motorActions.intakeGrabUntil(Enums.DetectedColor.UNKNOWN))
                */
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup2Task);

        PathChainTask score2Task = new PathChainTask(score2, 0.5)
                /*
                .addWaitAction(
                    () -> motorControl.lift.closeEnough(770),
                    new SequentialAction(
                        motorActions.outtakeLinkage.Retract(),
                        new SleepAction(0.4),
                        motorActions.outtakeClaw.Open(),
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Transfer()
                    )
                )
                */
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(score2Task);

        PathChainTask pickup3Task = new PathChainTask(intake3, 0.5)
                .setMaxWaitTime(1.25)
                /*
                .addWaitAction(1, motorActions.intakeGrabUntil(Enums.DetectedColor.UNKNOWN))
                */
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup3Task);

        PathChainTask score3Task = new PathChainTask(score3, 0.5)
                /*
                .addWaitAction(
                    () -> motorControl.lift.closeEnough(770),
                    new SequentialAction(
                        motorActions.outtakeLinkage.Retract(),
                        new SleepAction(0.4),
                        motorActions.outtakeClaw.Open(),
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Transfer()
                    )
                )
                */
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(score3Task);
    }

    @Override
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected double getCurrentTValue() {
        return follower.getCurrentTValue();
    }

    @Override
    protected void startPath(PathChainTask task) {
        follower.followPath((PathChain) task.pathChain, true);
    }

    @Override
    public void init() {
        super.init();
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        limelight     = new MotorControl.Limelight(hardwareMap, telemetry, 1);
        follower      = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        Servo ptor,ptol, sweeper;
        ptor           = hardwareMap.get(Servo.class, "ptor");
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        ptol           = hardwareMap.get(Servo.class, "ptol");
        ptol.setPosition(0.44);
        ptor.setPosition(0.60);
        sweeper.setPosition(0.67);

        /*
        run(motorActions.outtakeClaw.Close());
        */

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        currentTaskIndex = 0;
        taskPhase        = 0;
        pathTimer.resetTimer();

        /*
        run(motorActions.intakePivot.Transfer());
        run(motorActions.outtakeLinkage.Extend());
        run(motorActions.intakeArm.Transfer());
        run(motorActions.outtakeSampleAuto());
        */
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        runTasks();
        motorControl.update();

        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase",      (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value",    follower.getCurrentTValue());
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.update();
    }
}
