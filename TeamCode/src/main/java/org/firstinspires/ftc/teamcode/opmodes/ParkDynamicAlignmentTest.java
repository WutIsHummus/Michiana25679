package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * ParkDynamicAlignmentTest extends PathChainAutoOpMode, leveraging the two-phase
 * DRIVING/WAITING logic, separate timers, and maxWaitTime for tasks that have a condition.
 *
 * Tasks:
 *   1) ParkTask:  Known path, short wait time.
 *   2) DynamicAlignmentTask:  Path is computed at runtime if limelight sees an offset.
 *       - Condition: Wait until color detected, or up to a max time.
 *   3) ShiftLeftRightTask:  If color is still unknown, shift x +/- 5 for up to 3 seconds.
 */
@Autonomous(name = "Park & Limelight Adjustment Test")
public class ParkDynamicAlignmentTest extends PathChainAutoOpMode {

    // Robot Hardware & Helpers
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;

    // Simple Poses
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose parkPose  = new Pose(5, 0, Math.toRadians(0));

    // PathChains
    private PathChain parkChain;

    // Override buildPathChains(): define known geometry
    @Override
    protected void buildPathChains() {
        parkChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0, run(motorActions.lift.vision()))
                .setZeroPowerAccelerationMultiplier(4)
                .build();
    }

    // A helper method to build a dynamic alignment path at runtime
    private PathChain computeDynamicPath() {
        Pose currentPose = follower.getPose();
        Vector2d limeAvg = limelight.getAveragePose();
        double rawOffset = limeAvg.x;
        double fallbackOffset = 0.0;
        if (rawOffset == 99.99) {
            telemetry.addData("Dynamic Align", "No valid limelight data; using fallback offset");
            rawOffset = fallbackOffset;
        }


        Pose targetPose = new Pose(currentPose.getX(), rawOffset, currentPose.getHeading());
        telemetry.addData("Dynamic Align: Raw Offset", rawOffset);
        telemetry.addData("Dynamic Align: Target Pose", targetPose);
        telemetry.update();

        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(currentPose), new Point(targetPose)))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)

                .build();
    }


    // Override buildTaskList(): define tasks with wait times & conditions
    @Override
    protected void buildTaskList() {
        tasks.clear();

        // 1) Park Task: use known path, short wait
        //    No overall condition => uses waitTime directly
        PathChainTask parkTask = new PathChainTask(parkChain, 1.0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.lift.vision(),
                        motorActions.lift.waitUntilFinished(),
                        (packet) -> {
                            limelight.startCollectingSamples();
                            return false;
                        }
                ))
                .addWaitAction(0.2, (packet) -> {
                    limelight.collectSamples();
                    return false;
                });
        // No overall condition => once 1s passes, the next task starts
        tasks.add(parkTask);

        PathChainTask dynamicTask = new PathChainTask(null, 0.2);
        tasks.add(dynamicTask);


    }

    // ---------- Overridden Methods from PathChainAutoOpMode ----------

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
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected double getCurrentTValue() {
        return follower.getCurrentTValue();
    }

    /**
     * startPath(...) is called at the beginning of the DRIVING phase.
     * If the pathChain is null (i.e. dynamic), we compute it now.
     * Otherwise, we just follow it.
     */
    @Override
    protected void startPath(PathChainTask task) {
        if (task.pathChain == null) {
            // compute dynamic path
            task.pathChain = computeDynamicPath();
        }
        // Now follow
        if (task.pathChain != null) {
            follower.followPath((PathChain) task.pathChain, true);
        }
    }

    // ---------- Standard Lifecycle Methods ----------
    @Override
    public void init() {
        super.init();

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);


        limelight = new MotorControl.Limelight(hardwareMap, telemetry);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        // Any immediate actions
        run(new SequentialAction(motorActions.ptolLatch.unlock(),motorActions.ptorLatch.unlock()));

        // Build known geometry & tasks
        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
        waitTimer.resetTimer();
        actionTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();  // calls runTasks() in PathChainAutoOpMode
        follower.update();
        motorControl.update();

        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value", getCurrentTValue());
        telemetry.addData("Wait Timer", waitTimer.getElapsedTimeSeconds());
        telemetry.addData("Action Timer", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("Color", motorControl.getDetectedColor());
        telemetry.addData("LL offset", limelight.getAveragePose());
        telemetry.update();
    }
}
