package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
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

import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * SubSampleAutoRed: Multi-cycle auto where RED/YELLOW are the *wanted* colors.
 * If SHIFT or ALIGN sees a wanted color (RED or YELLOW), we:
 *   1) Stop SHIFT/ALIGN immediately.
 *   2) Run a big sequential action (intakeTransfer, etc.).
 *   3) Jump to the next scoring task (scoreSub1Task / scoreSub2Task).
 *
 * Updated to support turning operations using the enhanced PathChainAutoOpMode.
 */

@Autonomous(name = "New bucket")
public class NewBucket extends PathChainAutoOpMode {

    // ---------------------------
    // Hardware and Helper Fields
    // ---------------------------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;

    // ---------------------------
    // Key Poses
    // ---------------------------
    private final Pose startPose   = new Pose(9, 111, Math.toRadians(270));
    private final Pose scorePose   = new Pose(18, 126, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(20, 124, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(20, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(24, 128, Math.toRadians(23));

    // Park poses
    private final Pose parkPose        = new Pose(62, 97, Math.toRadians(270));
    private final Pose parkControlPose = new Pose(70, 120, Math.toRadians(270));

    // ---------------------------
    // PathChains
    // ---------------------------
    private PathChain scorePreload;
    private PathChain intake1, intake2, intake3;
    private PathChain score1, score2, score3;
    private PathChain parkChain;

    // ---------------------------
    // Extra tasks for multiple cycles
    // (We'll store references for SHIFT/ALIGN tasks)
    // ---------------------------
    private BaseTask dynamicTask, shiftTask;
    private BaseTask scoreSub1Task, park2Task, align2Task, shiftTask2, scoreSub2Task;




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
                        motorActions.outtakeSampleAuto()
                )))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading(), 50)
                .addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(10),
                        motorActions.outtakeSampleAuto()
                )))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading(), 50)
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(10),
                        motorActions.outtakeSampleAuto()
                )))
                .build();

        // ============ Preload path ============
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePose)))
                .addParametricCallback(0, () -> run(motorActions.outtakeSampleAuto()))
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

        // 1) Preload scoring
        PathChainTask preloadTask = new PathChainTask(scorePreload, 0.35)
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(800));
        tasks.add(preloadTask);

        // 2) pickup1 -> score1
        PathChainTask pickup1Task = new PathChainTask(intake1, 0.2)
                .setMaxWaitTime(1.25)
                .addWaitAction(1, motorActions.intakeTransfer())
                .addWaitAction(0.3, motorActions.extendo.set(500))
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup1Task);

        PathChainTask score1Task = new PathChainTask(score1, 0.4)
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(800));
        tasks.add(score1Task);

        // 3) pickup2 -> score2
        PathChainTask pickup2Task = new PathChainTask(intake2, 0.2)
                .setMaxWaitTime(1.25)
                .addWaitAction(1, motorActions.intakeTransfer())
                .addWaitAction(0, motorActions.extendo.set(500))
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup2Task);

        PathChainTask score2Task = new PathChainTask(score2, 0.4)
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(800));
        tasks.add(score2Task);

        // 4) pickup3 -> score3
        PathChainTask pickup3Task = new PathChainTask(intake3, 0.2)
                .setMaxWaitTime(1.25)
                .addWaitAction(1, motorActions.intakeTransfer())
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup3Task);

        PathChainTask score3Task = new PathChainTask(score3, 0.4)
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(800));
        tasks.add(score3Task);

        // 5) Park #1
        PathChainTask parkTask = new PathChainTask(parkChain, 0.2);
        tasks.add(parkTask);

        // 6) Dynamic Alignment #1 (null => computeDynamicPath)
        dynamicTask = new PathChainTask(null, 0.2);
        dynamicTask.addWaitAction(0, motorActions.extendo.set(50));
        dynamicTask.addWaitAction(0, motorActions.extendo.set(100));
        tasks.add(dynamicTask);

        // 7) SHIFT Task #1
        shiftTask = new PathChainTask(null, 0.0)
                // WaitCondition for SHIFT if color != UNKNOWN && color != RED
                .setWaitCondition(() ->
                        motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN
                                && motorControl.getDetectedColor() != Enums.DetectedColor.RED
                )
                .setMaxWaitTime(3.0)
                .addWaitAction(0, motorActions.inPivot.sampleGrab());

        shiftTask.addWaitAction(0,   motorActions.extendo.set(150));
        shiftTask.addWaitAction(0.6, motorActions.extendo.set(400));
        shiftTask.addWaitAction(
                () -> motorControl.getDetectedColor() == Enums.DetectedColor.RED,
                new SequentialAction(
                        shiftX(-4.0),
                        new SleepAction(0.5),
                        motorActions.extendo.set(375),
                        shiftX(+2.0),
                        new SleepAction(0.5),
                        motorActions.extendo.set(450)
                )
        );
        shiftTask.addWaitAction(
                2,
                new SequentialAction(
                        shiftX(-2.0),
                        new SleepAction(0.5),
                        shiftX(+2.0),
                        new SleepAction(0.5)
                )
        );
        tasks.add(shiftTask);

        // 8) ScoreSub1
        scoreSub1Task = new PathChainTask(null, 0.5)
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(800));
        tasks.add(scoreSub1Task);

        // 9) Park2
        park2Task = new PathChainTask(null, 0.15);
        tasks.add(park2Task);

        // 10) Align2
        align2Task = new PathChainTask(null, 0);
        align2Task.addWaitAction(0, motorActions.extendo.set(25));
        tasks.add(align2Task);

        // 11) SHIFT2
        shiftTask2 = new PathChainTask(null, 0.0)
                .setWaitCondition(() ->
                        motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN
                                && motorControl.getDetectedColor() != Enums.DetectedColor.RED
                )
                .setMaxWaitTime(3.0);
        shiftTask2.addWaitAction(0,   motorActions.extendo.set(100));
        shiftTask2.addWaitAction(0.6, motorActions.extendo.set(400));
        shiftTask2.addWaitAction(
                () -> motorControl.getDetectedColor() == Enums.DetectedColor.RED,
                new SequentialAction(
                        shiftX(-4.0),
                        new SleepAction(0.5),
                        motorActions.extendo.set(375),
                        shiftX(+2.0),
                        new SleepAction(0.5),
                        motorActions.extendo.set(450)
                )
        );
        shiftTask2.addWaitAction(
                2,
                new SequentialAction(
                        shiftX(-2.0),
                        new SleepAction(0.5),
                        shiftX(+2.0),
                        new SleepAction(0.5)
                )
        );
        tasks.add(shiftTask2);

        // 12) ScoreSub2
        scoreSub2Task = new PathChainTask(null, 0.5)
                .setMaxWaitTime(2)
                .addWaitAction(2, (packet) -> {
                    requestOpModeStop();
                    return false;
                })
                .setWaitCondition(() -> motorControl.lift.closeEnough(800));
        tasks.add(scoreSub2Task);
    }


    @Override
    protected void startPath(PathChainTask task) {
        if (task.pathChain == null) {
            if (task == scoreSub1Task) {
                task.pathChain = buildShiftToScorePath();
            } else if (task == park2Task) {
                task.pathChain = buildScoreToParkPath();
            } else if (task == align2Task) {
                task.pathChain = computeDynamicPath();
            } else if (task == shiftTask2) {
                task.pathChain = computeDynamicPath();
            } else if (task == scoreSub2Task) {
                task.pathChain = buildShiftToScorePath();
            } else {
                // dynamicTask or shiftTask #1
                task.pathChain = computeDynamicPath();
            }
        }
        if (task.pathChain != null) {
            follower.followPath((PathChain) task.pathChain, true);
        }
    }

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

    // ---------------------------------------------------
    // CANCEL SHIFT/ALIGN IF RED/YELLOW (Wanted Colors)
    // ---------------------------------------------------
    @Override
    protected void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            return; // done
        }
        // Possibly cancel SHIFT/ALIGN early if we see red/yellow
        checkEarlyColorStop();

        // Use the enhanced base class logic
        super.runTasks();
    }

    /**
     * If SHIFT or ALIGN sees a *wanted* color (RED or YELLOW), we:
     * 1) stop the SHIFT/ALIGN path,
     * 2) run a big sequential action (intakeTransfer, etc.),
     * 3) jump to next scoring.
     */
    private void checkEarlyColorStop() {
        Enums.DetectedColor color = motorControl.getDetectedColor();
        BaseTask currentTask = tasks.get(currentTaskIndex);

        // SHIFT or ALIGN tasks
        boolean isShiftOrAlign = (currentTask == shiftTask || currentTask == dynamicTask
                || currentTask == shiftTask2 || currentTask == align2Task);

        // If we see RED or YELLOW (our "wanted" colors) in SHIFT/ALIGN
        if (isShiftOrAlign && (color == Enums.DetectedColor.BLUE || color == Enums.DetectedColor.YELLOW)) {
            // 1) Stop the path
            stopCurrentPath();

            markAllTriggered(currentTask);

            // 2) run the big sequential action
           /*run(new SequentialAction(
                    motorActions.intakeTransfer(),
                    motorActions.extendo.waitUntilFinished(0),
                    new SleepAction(0.3),
                    motorActions.outtakeSampleAuto(),
                    motorActions.spin.poop()
            ));*/

            // 3) Jump to next scoring. SHIFT #1 => scoreSub1Task, SHIFT #2 => scoreSub2Task
            if (currentTask == shiftTask) {
                currentTaskIndex = tasks.indexOf(scoreSub1Task);
            } else if (currentTask == shiftTask2) {
                currentTaskIndex = tasks.indexOf(scoreSub2Task);
            } else {
                // If it's an alignment task, jump to the next SHIFT or first scoring
                currentTaskIndex = tasks.indexOf(scoreSub1Task);
            }
            // 4) Reset phase
            taskPhase = 0;
        }
    }

    private void markAllTriggered(BaseTask task) {
        for (WaitAction wa : task.waitActions) {
            wa.triggered = true;
        }
    }

    private void stopCurrentPath() {
        follower.breakFollowing();
    }

    // ---------------------------
    // SHIFT->SCORE, PARK, etc.
    // ---------------------------
    private PathChain buildShiftToScorePath() {
        Pose curPose = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(curPose),new Point(parkControlPose), new Point(scorePose)))
                /*.addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakeTransfer(),
                        motorActions.extendo.waitUntilFinished(0),
                        new SleepAction(0.5),
                        motorActions.outtakeSampleAuto(),motorActions.spin.slowpoop()
                )))*/
                //.addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .setLinearHeadingInterpolation(curPose.getHeading(), scorePose.getHeading())
                .build();
    }

    private PathChain buildScoreToParkPath() {
        Pose curPose = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(curPose), new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(curPose.getHeading(), parkPose.getHeading())
                .build();
    }

    protected PathChain computeDynamicPath() {
        Pose cur = follower.getPose();
        double rawYaw = limelight.getDistance() != null
                ? limelight.getDistance().yawDegrees
                : 0.0;
        double xShift = rawYaw * 1.5;
        Pose target = new Pose(
                Math.min(85, Math.max(60, cur.getX() + xShift)),
                cur.getY(),
                cur.getHeading()
        );
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(cur), new Point(target)))
                .setConstantHeadingInterpolation(target.getHeading())
                .build();
    }


    // ---------------------------
    // Required Follower Integration Methods
    // ---------------------------
    @Override
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected boolean isTurning() {
        return follower.isTurning();
    };

    @Override
    protected double getCurrentTValue() {
        return follower.getCurrentTValue();
    }

    @Override
    public void init() {
        super.init();
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        limelight = new MotorControl.Limelight(hardwareMap, telemetry);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        // Set turn error threshold if desired (uncomment and adjust as needed)
        // FConstants.FollowerConstants.turnHeadingErrorThreshold = 0.05; // default is 0.01

        run(motorActions.safePositions());

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();

        //run(motorActions.outtakeSampleAuto());
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        motorControl.update();

        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE/TURN" : "WAIT");
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("Path Active", isPathActive());
        telemetry.addData("Turning", isTurning());
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.addData("Detected Color", motorControl.getDetectedColor());
        telemetry.addData("Limelight Avg", limelight.getAveragePose());
        telemetry.update();
    }
}