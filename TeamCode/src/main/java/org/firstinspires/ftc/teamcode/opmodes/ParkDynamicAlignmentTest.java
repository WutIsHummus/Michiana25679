package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action; // Keep for potential future WaitActions
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction; // Keep if used by motorActions
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo; // Added from your latest LimelightAlignmentTest

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater; // Added from your latest LimelightAlignmentTest
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker; // Added from your latest LimelightAlignmentTest
import com.pedropathing.util.Drawing; // Added from your latest LimelightAlignmentTest

import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Park & Limelight V2 (Corrected Sampling)")
public class ParkDynamicAlignmentTest extends PathChainAutoOpMode {

    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;

    // For dashboard drawing
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose parkPose  = new Pose(5, 0, Math.toRadians(0)); // Robot moves 5 units along Field X

    private PathChain parkChain;

    // Task references for clarity
    private PathChainTask parkAndPrepareLimelightTask;
    private PathChainTask dynamicAlignmentExecutionTask;
    // private PathChainTask shiftTask; // Still here but not used in this simplified version

    @Override
    protected void buildPathChains() {
        parkChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                // Parametric callback to start Limelight sampling as soon as parkChain begins
                .addParametricCallback(0.01, () -> { // t=0.01 to ensure path has started
                    telemetry.addLine("Parametric CB: Starting Limelight sample collection prep.");
                    limelight.startCollectingSamples(); // Resets buffers and sets isCollectingSamples = true
                })
                .addParametricCallback(0, ()-> run(motorActions.lift.vision()))
                .setZeroPowerAccelerationMultiplier(4)
                .build();
    }

    // This method will be called by startPath for dynamicTask
    private PathChain computeAndBuildDynamicPath() {
        Pose currentPose = follower.getPose();
        if (Double.isNaN(currentPose.getX()) || Double.isNaN(currentPose.getY()) || Double.isNaN(currentPose.getHeading())) {
            telemetry.log().add("ERROR: Follower pose NaN in computeAndBuildDynamicPath. Using (0,0,0).");
            currentPose = new Pose(0,0,0);
        }

        Vector2d limeAvg = limelight.getAveragePose(); // (HorizontalError, ForwardDistance)
        double rawHorizontalError = limeAvg.x; // This is your LL_horz

        telemetry.addData("ComputeDynamicPath", "Using LL H_Error: %.2f", rawHorizontalError);

        if (rawHorizontalError == 99.99) { // Check for invalid Limelight data
            telemetry.addData("Dynamic Align", "No valid limelight data; creating dummy path (stay still).");
            return follower.pathBuilder()
                    .addPath(new BezierLine(new Point(currentPose), new Point(currentPose)))
                    .build();
        }

        // Your original logic for target pose based on rawOffset (LL horizontal error)
        // This moves the robot TO Field Y = rawHorizontalError, keeping current Field X and Heading.
        // This is NOT a robot-centric strafe. It's a move to a specific Y-line on the field.
        Pose targetPose = new Pose(currentPose.getX(), -rawHorizontalError, currentPose.getHeading());

        telemetry.addData("Dynamic Align", "Raw Offset (LL Horiz): %.2f", rawHorizontalError);
        telemetry.addData("Dynamic Align", "Current Pose: %s", currentPose.toString());
        telemetry.addData("Dynamic Align", "Computed Target Field Pose: %s", targetPose.toString());

        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(currentPose), new Point(targetPose)))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Task 1: Execute the predefined parkChain.
        // The parametric callback in parkChain calls limelight.startCollectingSamples().
        // Wait time here is for *after* the path physically completes.
        parkAndPrepareLimelightTask = new PathChainTask(parkChain, 0.1); // Short wait after path
        tasks.add(parkAndPrepareLimelightTask);

        // Task 2: Dynamic Alignment Task.
        // This task, in its startPath, will first ensure Limelight samples are collected,
        // then compute its own path using that data.
        // The waitTime is the max duration for the dynamically generated path to execute.
        dynamicAlignmentExecutionTask = new PathChainTask(null, 3.0);
        dynamicAlignmentExecutionTask.setWaitCondition(() -> !follower.isBusy()); // Completes when its dynamic path is done
        tasks.add(dynamicAlignmentExecutionTask);

        // shiftTask is not being used in this simplified flow for now.
        // PathChainTask shiftTask = new PathChainTask(null, 0.0);
        // tasks.add(shiftTask);
    }

    @Override
    protected void startPath(PathChainTask task) {
        Pose currentPose = follower.getPose(); // Get current pose at the start of any path execution
        if (Double.isNaN(currentPose.getX()) || Double.isNaN(currentPose.getY()) || Double.isNaN(currentPose.getHeading())) {
            telemetry.log().add("CRITICAL ERROR: Follower pose NaN in startPath. OpMode may behave erratically. Pose set to (0,0,0).");
            currentPose = new Pose(0,0,0); // Fallback
        }

        if (task == parkAndPrepareLimelightTask) {
            // Path (parkChain) is already defined in the task object by buildTaskList & buildPathChains
            telemetry.addLine("Task: Starting Park Path & Preparing Limelight...");
            if (task.pathChain != null) { // Should be parkChain
                follower.followPath((PathChain) task.pathChain, true);
            } else { // Fallback, should not happen
                telemetry.log().add("Error: parkAndPrepareLimelightTask.pathChain was null!");
                task.pathChain = follower.pathBuilder().addPath(new BezierLine(new Point(currentPose), new Point(currentPose))).build();
                follower.followPath((PathChain) task.pathChain, true);
            }
        } else if (task == dynamicAlignmentExecutionTask) {
            telemetry.addLine("Task: Dynamic Alignment - Collecting Limelight Samples...");
            // Ensure startCollectingSamples was called (e.g., by parkChain's parametric callback)
            // If not, or to be safe for retries:
            limelight.startCollectingSamples();

            long samplingStartTime = System.currentTimeMillis();
            boolean samplesCollectedSuccessfully = false;
            while (opModeIsActive() && (System.currentTimeMillis() - samplingStartTime) < 1500) { // 1.5s timeout for LL
                if (limelight.collectSamples()) { // collectSamples runs the full PipeA->PipeB logic
                    if (limelight.getAveragePose().x != 99.99) { // Check for valid data marker
                        samplesCollectedSuccessfully = true;
                        telemetry.addLine("Limelight samples collected successfully.");
                        break;
                    }
                }
                // Brief yield for other processes if this were LinearOpMode; PathChainAutoOpMode handles loop.
                // If this loop runs too long, it might starve the main OpMode loop.
                // For this test, it's acting as a blocking read within startPath.
            }

            if (!samplesCollectedSuccessfully) {
                telemetry.addLine("Limelight: Failed to collect valid samples for dynamic path.");
            }
            // `computeAndBuildDynamicPath` will use `limelight.getAveragePose()` which now holds fresh (or default 99.99) data
            task.pathChain = computeAndBuildDynamicPath(); // This will build the actual path to follow

            if (task.pathChain != null) {
                telemetry.addLine("Task: Starting dynamically computed alignment path...");
                follower.followPath((PathChain) task.pathChain, true);
            } else { // Should not happen if computeAndBuildDynamicPath always returns a path
                telemetry.log().add("Error: dynamicAlignmentExecutionTask.pathChain ended up null after compute.");
                // Create a dummy path to prevent crash
                task.pathChain = follower.pathBuilder().addPath(new BezierLine(new Point(currentPose), new Point(currentPose))).build();
                follower.followPath((PathChain) task.pathChain, true);
            }
        }
        // Add handling for shiftTask if you re-introduce it
    }

    @Override
    protected void startTurn(TurnTask task) {
        // Standard turn logic from PathChainAutoOpMode
        if (task.isRelative) {
            if (task.useDegrees) follower.turnDegrees(task.angle, task.isLeft);
            else follower.turn(task.angle, task.isLeft);
        } else {
            if (task.useDegrees) follower.turnToDegrees(task.angle);
            else follower.turnTo(task.angle);
        }
    }

    @Override
    protected boolean isTurning() { return follower.isTurning(); }
    @Override
    protected boolean isPathActive() { return follower.isBusy(); }
    @Override
    protected double getCurrentTValue() {
        if (follower.isBusy() && follower.getCurrentPath() != null && follower.getCurrentPath().length() > 0) {
            double tVal = follower.getCurrentTValue();
            return Double.isNaN(tVal) ? 0.0 : tVal;
        }
        return 0.0;
    }

    @Override
    public void init() {
        super.init();
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        limelight = new MotorControl.Limelight(hardwareMap, telemetry);
        limelight.setPrimaryClass("blue"); // Set your primary target color


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        run(motorActions.safePositions());
        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start(); // Resets currentTaskIndex, taskPhase, timers
        // poseUpdater and dashboardPoseTracker are initialized in init()
    }

    @Override
    public void loop() {
        super.loop(); // Handles task execution via runTasks()
        follower.update();
        motorControl.update();
        poseUpdater.update();


        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size())
                .addData("Phase", (taskPhase == 0) ? "DRIVE/TURN" : "WAIT")
                .addData("T Value", String.format("%.3f", getCurrentTValue()))
                .addData("LL Raw AvgPose", limelight.getAveragePose().toString()) // Display what LL class is holding
                .addData("Follower Pose", follower.getPose().toString());

        if (poseUpdater != null) {
            Pose p = poseUpdater.getPose();
            if (!Double.isNaN(p.getX())) { // Basic check for valid pose data
                Drawing.drawRobot(p, "#4CAF50");
                if (dashboardPoseTracker != null) {
                    dashboardPoseTracker.update();
                    Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
                }
                Drawing.sendPacket();
            }
        }
        telemetry.update();
    }
}