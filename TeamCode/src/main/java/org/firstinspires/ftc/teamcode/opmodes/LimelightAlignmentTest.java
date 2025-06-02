package org.firstinspires.ftc.teamcode.opmodes;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Limelight Align V7 (X-Fwd, Trig Strafe)", group = "Test") // Name might need update if behavior changes
// @Disabled // Remove this line to show in OpMode list
public class LimelightAlignmentTest extends PathChainAutoOpMode {

    private Follower follower;
    private MotorControl motorControl;
    private MotorActions motorActions;
    private MotorControl.Limelight limelight;

    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;


    // --- Alignment Constants (Tune these!) ---
    private static final String PRIMARY_TARGET_COLOR = "blue";

    private static final double HORIZONTAL_ERROR_THRESHOLD = 1.0;
    private static final double ANGLE_ERROR_THRESHOLD = 2.0;

    // Gains:
    // STRAFE_GAIN_INCHES_PER_LL_HORIZ_ERROR is used in the trig version.
    // For the direct field Y modification in your current code, the "gain" is effectively -1 or +1.
    private static final double STRAFE_GAIN_DIRECT_Y_MODIFICATION = -1.0; // Assuming latestHorizontalError positive means target right, and you want to decrease Field Y
    private static final double TURN_GAIN_DEGREES_PER_LL_ANGLE_ERROR = 1.0; // User code has this gain

    private double latestHorizontalError = 0;
    private double latestAngleError = 0;
    private boolean limelightDataValid = false;

    private PathChainTask limelightReadAndDecideTask;
    private PathChainTask executeStrafeTask;
    private TurnTask executeTurnTask;

    private Pose targetStrafePose = new Pose(0,0,0);

    @Override
    public void init() {
        super.init();

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        limelight = new MotorControl.Limelight(hardwareMap, telemetry);
        limelight.setPrimaryClass(PRIMARY_TARGET_COLOR);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        // Initialize PoseUpdater and DashboardPoseTracker
        poseUpdater = new PoseUpdater(hardwareMap); // From your snippet
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater); // From your snippet


        buildPathChains();
        buildTaskList();
    }

    @Override
    protected void buildPathChains() {
        // No predefined paths for this OpMode
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();
        limelightReadAndDecideTask = new PathChainTask(null, 3);
        tasks.add(limelightReadAndDecideTask);

        executeStrafeTask = new PathChainTask(null, 0.1) // waitTime is for after path completes
                .setWaitCondition(() -> !follower.isBusy())
                .setMaxWaitTime(3.0);

        tasks.add(executeStrafeTask);
    }

    @Override
    protected void startPath(PathChainTask task) {
        if (task == limelightReadAndDecideTask) {
            telemetry.addLine("Task: Reading Limelight...");
            limelight.startCollectingSamples();
            long startTime = System.currentTimeMillis();
            boolean samplesCollected = false;

            while (opModeIsActive() && (System.currentTimeMillis() - startTime) < 1000) {
                if (limelight.collectSamples()) {
                    samplesCollected = true;
                    break;
                }
                // In ActionOpMode, sleep() is not ideal here. The main loop handles yielding.
            }

            if (samplesCollected) {
                latestHorizontalError = limelight.getAveragePose().x;
                latestAngleError = limelight.getAverageAngle();
                limelightDataValid = (latestHorizontalError != 99.99 && latestAngleError != 99.99);
                if (limelightDataValid) {
                    telemetry.addData("Limelight Read", "H_Error: %.2f, Angle_Error: %.2f", latestHorizontalError, latestAngleError);
                } else {
                    telemetry.addLine("Limelight: Got default/error values (99.99). No valid target?");
                }
            } else {
                limelightDataValid = false;
                telemetry.addLine("Limelight: Failed to collect samples in time.");
            }
            limelight.resetSamples();

            Pose currentPose = follower.getPose();
            if (Double.isNaN(currentPose.getX()) || Double.isNaN(currentPose.getY()) || Double.isNaN(currentPose.getHeading())) {
                telemetry.log().add("ERROR: Follower pose has NaN values. Defaulting to 0,0,0 for tiny path.");
                currentPose = new Pose(0,0,0);
            }

            // User's "tiny path" - robot moves 5 inches along Field X
            Pose tinyTargetPose = new Pose(currentPose.getX(), currentPose.getY() + 2, currentPose.getHeading());
            PathChain tinyPath = follower.pathBuilder().addPath(new BezierLine(
                            new Point(currentPose),
                            new Point(tinyTargetPose)))
                    .addParametricCallback(0, ()-> run(motorActions.lift.vision()))
                    .build();
            task.pathChain = tinyPath;
            follower.followPath((PathChain) task.pathChain, true);

        } else if (task == executeStrafeTask) {
            if (task.pathChain != null) {
                telemetry.addLine("Task: Executing strafe path..."); // Updated telemetry
                follower.followPath((PathChain) task.pathChain, true);
            } else {
                telemetry.addLine("Error: executeStrafeTask had no path. Creating dummy.");
                Pose currentPose = follower.getPose();
                if (Double.isNaN(currentPose.getX()) || Double.isNaN(currentPose.getY()) || Double.isNaN(currentPose.getHeading())) {
                    currentPose = new Pose(0,0,0);
                }
                task.pathChain = follower.pathBuilder().addPath(new BezierLine(
                                new Point(currentPose),
                                new Point(currentPose)))
                        .build();
                follower.followPath((PathChain)task.pathChain, true);
            }
        }
    }

    @Override
    protected void startTurn(TurnTask task) {
        if (task == executeTurnTask) {
            telemetry.addData("Task: Executing dynamic turn. Commanded Angle:", String.format("%.2f deg. IsLeft: %s", task.angle, task.isLeft));
            follower.turnDegrees(task.angle, task.isLeft);
        }
    }

    @Override
    protected void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            if (gamepad1.a && opModeIsActive()) {
                telemetry.addLine("Re-running alignment task (A pressed).");
                currentTaskIndex = 0;
                taskPhase = 0;
                limelightDataValid = false;
                tasks.clear();
                buildTaskList(); // This will repopulate tasks with limelightReadAndDecideTask at index 0
                if (!tasks.isEmpty()) {
                    tasks.get(0).resetWaitActions();
                }
            } else {
                if (opModeIsActive()) {
                    telemetry.addLine("Alignment sequence complete. Press A to run again.");
                }
                return; // Done with tasks or waiting for 'A'
            }
        }

        super.runTasks(); // Handles current task execution and state transitions

        // This logic block executes AFTER limelightReadAndDecideTask has finished its "tiny path"
        // AND its short waitTime (0.05s) has elapsed (meaning taskPhase will be 1 for it WHEN it's current).
        if (currentTaskIndex < tasks.size() && tasks.get(currentTaskIndex) == limelightReadAndDecideTask && taskPhase == 1) {

            if (currentTaskIndex >= tasks.size()) return; // Safety check

            if (!limelightDataValid) {
                telemetry.addLine("Limelight data invalid after read task. Ending current alignment cycle.");
                if (tasks.get(currentTaskIndex) == limelightReadAndDecideTask) { // Check if it's *still* this task
                    currentTaskIndex = tasks.size(); // Force end of tasks
                }
                return;
            }

            boolean actionTaken = false;
            Pose currentPose = follower.getPose();
            double currentHeadingRad = currentPose.getHeading(); // Storing to avoid repeated calls
            if (Double.isNaN(currentPose.getX()) || Double.isNaN(currentPose.getY()) || Double.isNaN(currentHeadingRad)) {
                telemetry.log().add("ERROR: Follower pose NaN in runTasks decision phase. Using 0,0,0.");
                currentPose = new Pose(0,0,0);
                currentHeadingRad = 0.0;
            }

            if (Math.abs(latestHorizontalError) > HORIZONTAL_ERROR_THRESHOLD) {
                telemetry.addData("Decision", "Strafe needed. H_Error: %.2f", latestHorizontalError);

                // User's current strafe logic from their provided code:
                // Modifies Field Y based on horizontal error.
                // Assumes STRAFE_GAIN_DIRECT_Y_MODIFICATION handles the scaling and sign.
                // Let's use a gain for clarity, assuming latestHorizontalError > 0 means target is RIGHT,
                // and robot needs to decrease its Field Y to move "right" on a typical field graph if it's sideways.
                // The sign of STRAFE_GAIN_DIRECT_Y_MODIFICATION will determine direction.
                double fieldYModification = latestHorizontalError;

                targetStrafePose = new Pose(
                        currentPose.getX(), // X field coordinate remains the same
                        currentPose.getY() + fieldYModification, // Modify Y field coordinate
                        currentHeadingRad // Maintain heading
                );

                // **Added Telemetry for targetStrafePose**
                telemetry.addData("Target Strafe Pose", targetStrafePose.toString());

                PathChain strafePath = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Point(currentPose),
                                new Point(targetStrafePose)))
                        .setLinearHeadingInterpolation(currentHeadingRad, currentHeadingRad)
                        .build();

                executeStrafeTask.pathChain = strafePath;
                tasks.set(currentTaskIndex, executeStrafeTask); // Replace current (limelightRead) task with this strafe task
                actionTaken = true;

            } else if (Math.abs(latestAngleError) > ANGLE_ERROR_THRESHOLD) {
                telemetry.addData("Decision", "Turn needed. Angle_Error: %.2f", latestAngleError);
                // Positive latestAngleError = Target "yawed" CW (from cam view), Robot needs CCW (positive) turn.
                double turnAmountDegrees = -latestAngleError;

                // **Added Telemetry for turnAmountDegrees**
                telemetry.addData("Calculated Turn", "%.2f degrees", turnAmountDegrees);

                executeTurnTask.angle = Math.abs(turnAmountDegrees);
                executeTurnTask.isLeft = turnAmountDegrees > 0;

                tasks.set(currentTaskIndex, executeTurnTask); // Replace current task with this turn task
                actionTaken = true;
            }

            if (actionTaken) {
                taskPhase = 0; // Reset to DRIVING/TURNING phase for the newly set task
                pathTimer.resetTimer();
                actionTimer.resetTimer();
                tasks.get(currentTaskIndex).resetWaitActions();
            } else {
                telemetry.addLine("Alignment OK or no further action taken. Ending current alignment cycle.");
                currentTaskIndex++; // No action taken, so advance past limelightReadAndDecideTask
            }
            limelightDataValid = false;
        }
    }

    @Override
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected boolean isTurning() {
        return follower.isTurning();
    }

    @Override
    protected double getCurrentTValue() {
        // Guard against NaN from follower if it's not on a valid path or path has zero length.
        if (follower.isBusy() && follower.getCurrentPath() != null ) {
            Path currentPath = follower.getCurrentPath();
            if (currentPath.length() > 0 ) {
                double tVal = follower.getCurrentTValue();
                return Double.isNaN(tVal) ? 0.0 : tVal;
            }
        }
        return 0.0;
    }

    @Override
    public void start() {
        super.start();
        limelightDataValid = false;
        // currentTaskIndex and taskPhase are reset by super.start()

        // Servo Init (from your snippet)
        Servo ptor,ptol, sweeper;
        ptor           = hardwareMap.get(Servo.class, "ptor");
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        ptol           = hardwareMap.get(Servo.class, "ptol");
        ptol.setPosition(0.44);
        ptor.setPosition(0.60);
        sweeper.setPosition(0.67);

        // Ensure task list is correctly set up for the first run
        // This check might be too simple if buildTaskList is complex, but for this OpMode it's okay
        if (tasks.isEmpty() || tasks.get(0) != limelightReadAndDecideTask) {
            buildTaskList();
        }
        if (!tasks.isEmpty()) { // Ensure tasks list isn't empty before accessing
            tasks.get(0).resetWaitActions();
        }
    }

    @Override
    public void loop() {
        super.loop(); // This calls runTasks()

        if(follower != null) follower.update(); // Ensure follower is updated to get latest pose
        if(poseUpdater != null) poseUpdater.update(); // Update this if it's separate from follower's internal

        telemetry.addData("Current Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Task Phase", (taskPhase == 0) ? "DRIVE/TURN" : "WAIT");
        if (currentTaskIndex < tasks.size()) {
            BaseTask currentTaskToDisplay = tasks.get(currentTaskIndex);
            if (currentTaskToDisplay instanceof PathChainTask && ((PathChainTask)currentTaskToDisplay).pathChain != null) {
                telemetry.addData("T Value", String.format("%.3f", getCurrentTValue()));
            }
            if (currentTaskToDisplay instanceof TurnTask) {
                telemetry.addData("Turn Cmd Telemetry", String.format("%.2f deg. IsLeft: %s",
                        ((TurnTask)currentTaskToDisplay).angle,
                        ((TurnTask)currentTaskToDisplay).isLeft));
            }
        }

        // Drawing from your snippet
        if (poseUpdater != null) {
            Pose currentDrawPose = poseUpdater.getPose(); // Use the pose from the updater you're drawing
            if(!Double.isNaN(currentDrawPose.getX())){
                Drawing.drawRobot(currentDrawPose, "#4CAF50");
                if (dashboardPoseTracker != null) {
                    dashboardPoseTracker.update(); // Update the tracker with the latest pose
                    Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
                }
                Drawing.sendPacket(); // If using FTC Dashboard stream
            }
        }

        telemetry.addData("Target Strafe Pose", targetStrafePose.toString());
        telemetry.addData("Follower Pose", follower != null ? follower.getPose().toString() : "Follower Null!");
        telemetry.addData("LL H_Error", String.format("%.2f", latestHorizontalError));
        telemetry.addData("LL A_Error", String.format("%.2f", latestAngleError));
        telemetry.addData("LL Data Valid", limelightDataValid);
        telemetry.addData("Press Gamepad A to Re-Align", "");
        telemetry.update();
    }
}