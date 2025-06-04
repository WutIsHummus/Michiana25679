package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket; // Required for Action interface
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Limelight Alignment Test (Corrected)", group = "Test")
public class LimelightAlignmentTest extends ActionOpMode {

    private Follower follower;
    private MotorControl motorControl;
    private MotorActions motorActions;
    private MotorControl.Limelight limelight;

    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    // --- Alignment Constants ---
    private static final String PRIMARY_TARGET_COLOR = "blue"; // Or "red", "yellow"
    private static final double HORIZONTAL_ERROR_THRESHOLD = 1.0; // Inches (or unit of LL H_Error)
    private static final double ANGLE_ERROR_THRESHOLD = 2.0;    // Degrees
    private static final double STRAFE_ADJUSTMENT_FACTOR = -0.75; // Adjust this gain. Negative if positive LL H_Error means target is to the right and you want to move robot's Y negatively.
    private static final double TURN_ADJUSTMENT_FACTOR = -0.75;   // Adjust this gain. Negative if positive LL Angle_Error means target is CW (from robot's perspective) and robot needs to turn CCW.

    private double latestHorizontalError = 0;
    private double latestAngleError = 0;
    private boolean limelightDataValid = false;

    private enum AlignmentState {
        IDLE,
        STARTING_ALIGNMENT,
        MOVING_FOR_VISIBILITY,
        READING_LIMELIGHT,
        EXECUTING_ADJUSTMENT,
        ALIGNED,
        FAILED
    }
    private AlignmentState currentState = AlignmentState.IDLE;
    private Action currentExecutingAction = null;
    private int alignmentAttempts = 0;
    private final int MAX_ALIGNMENT_ATTEMPTS = 5;
    private boolean gamepadAWasPressed = false;

    // --- Action Wrapper Classes ---
    public static class PathFollowingAction implements Action {
        private Follower follower;
        private PathChain path;
        private boolean pathStarted = false;

        public PathFollowingAction(Follower follower, PathChain path) {
            this.follower = follower;
            this.path = path;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!pathStarted) {
                follower.followPath(path, true); // autonomous_mode = true
                pathStarted = true;
            }
            return follower.isBusy(); // True if still busy, false when done
        }
    }

    public static class RelativeTurnDegreesAction implements Action {
        private Follower follower;
        private double degreesMagnitude;
        private boolean turnLeft;
        private boolean turnStarted = false;

        public RelativeTurnDegreesAction(Follower follower, double degreesMagnitude, boolean turnLeft) {
            this.follower = follower;
            this.degreesMagnitude = degreesMagnitude;
            this.turnLeft = turnLeft;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!turnStarted) {
                follower.turnDegrees(degreesMagnitude, turnLeft);
                turnStarted = true;
            }
            return follower.isTurning(); // True if still turning, false when done
        }
    }


    @Override
    public void init() {
        super.init();
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        limelight = new MotorControl.Limelight(hardwareMap, telemetry);
        limelight.setPrimaryClass(PRIMARY_TARGET_COLOR);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        run(motorActions.safePositions());


        telemetry.addLine("Limelight Alignment Test (Corrected)");
        telemetry.addLine("Press Gamepad A to start alignment cycle.");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        currentState = AlignmentState.IDLE;

        Servo ptor = hardwareMap.get(Servo.class, "ptor");
        Servo ptol = hardwareMap.get(Servo.class, "ptol");
        Servo sweeper = hardwareMap.get(Servo.class, "sweeper");
        ptol.setPosition(0.44);
        ptor.setPosition(0.60);
        sweeper.setPosition(0.67);
    }

    @Override
    public void loop() {
        super.loop();

        if (gamepad1.a && !gamepadAWasPressed) {
            if (currentState == AlignmentState.IDLE || currentState == AlignmentState.ALIGNED || currentState == AlignmentState.FAILED) {
                currentState = AlignmentState.STARTING_ALIGNMENT;
                alignmentAttempts = 0;
                if (currentExecutingAction != null && runningActions.contains(currentExecutingAction)) {
                    runningActions.remove(currentExecutingAction);
                }
                currentExecutingAction = null;
            }
        }
        gamepadAWasPressed = gamepad1.a;

        switch (currentState) {
            case IDLE:
                // Waiting for 'A' press
                break;

            case STARTING_ALIGNMENT:
                telemetry.addLine("State: STARTING_ALIGNMENT -> MOVING_FOR_VISIBILITY");
                Pose currentPoseForVisibility = follower.getPose();
                // Minimal move, e.g., 0.5 units in current Y (relative forward if robot Y is fwd)
                Pose visibilityTargetPose = new Pose(currentPoseForVisibility.getX(), currentPoseForVisibility.getY() + 0.5, currentPoseForVisibility.getHeading());
                PathChain visibilityPath = follower.pathBuilder().addPath(new BezierLine(new Point(currentPoseForVisibility), new Point(visibilityTargetPose))).build();
                currentExecutingAction = new PathFollowingAction(follower, visibilityPath);
                run(currentExecutingAction);
                currentState = AlignmentState.MOVING_FOR_VISIBILITY;
                break;

            case MOVING_FOR_VISIBILITY:
                if (currentExecutingAction == null || !runningActions.contains(currentExecutingAction)) {
                    telemetry.addLine("State: MOVING_FOR_VISIBILITY -> READING_LIMELIGHT");
                    currentState = AlignmentState.READING_LIMELIGHT;
                }
                break;

            case READING_LIMELIGHT:
                Action readLLAction = packet -> {
                    telemetry.addLine("State: READING_LIMELIGHT - Collecting Samples...");
                    limelight.startCollectingSamples();
                    long samplingStartTime = System.currentTimeMillis();
                    boolean samplesCollectedSuccessfully = false;
                    while (opModeIsActive() && (System.currentTimeMillis() - samplingStartTime) < 1000) {
                        if (limelight.collectSamples()) {
                            if (limelight.getAveragePose().x != 99.99) {
                                samplesCollectedSuccessfully = true;
                                break;
                            }
                        }
                    }

                    if (samplesCollectedSuccessfully) {
                        latestHorizontalError = limelight.getAveragePose().x;
                        latestAngleError = limelight.getAverageAngle();
                        limelightDataValid = true;
                        telemetry.addData("LL Data OK", "H_Error: %.2f, Angle_Error: %.2f", latestHorizontalError, latestAngleError);
                    } else {
                        limelightDataValid = false;
                        telemetry.addLine("LL Data FAILED or Timed Out in READ_LIMELIGHT action.");
                    }
                    limelight.resetSamples();

                    // Transition logic based on read result, directly in the action's completion
                    if (!limelightDataValid) {
                        alignmentAttempts++;
                        currentState = (alignmentAttempts >= MAX_ALIGNMENT_ATTEMPTS) ? AlignmentState.FAILED : AlignmentState.STARTING_ALIGNMENT; // Retry
                    } else if (Math.abs(latestHorizontalError) > HORIZONTAL_ERROR_THRESHOLD || Math.abs(latestAngleError) > ANGLE_ERROR_THRESHOLD) {
                        currentState = AlignmentState.EXECUTING_ADJUSTMENT;
                    } else {
                        currentState = AlignmentState.ALIGNED;
                    }
                    return false; // This action part is done
                };
                currentExecutingAction = readLLAction; // Assign to currentExecutingAction
                run(currentExecutingAction); // Schedule the action
                currentState = AlignmentState.IDLE; // Temporarily go to IDLE, action callback will set the next state
                break;


            case EXECUTING_ADJUSTMENT:
                if (currentExecutingAction == null || !runningActions.contains(currentExecutingAction)) {
                    telemetry.addLine("State: EXECUTING_ADJUSTMENT - Calculating new adjustment");
                    Pose currentPose = follower.getPose();
                    Action adjustmentAction = null;

                    if (Math.abs(latestHorizontalError) > HORIZONTAL_ERROR_THRESHOLD) {
                        telemetry.addData("Adjustment", "Strafe for H_Error: %.2f", latestHorizontalError);
                        double fieldYModification = latestHorizontalError * STRAFE_ADJUSTMENT_FACTOR;
                        Pose targetStrafePose = new Pose(currentPose.getX(), currentPose.getY() + fieldYModification, currentPose.getHeading());
                        PathChain strafePath = follower.pathBuilder()
                                .addPath(new BezierLine(new Point(currentPose), new Point(targetStrafePose)))
                                .setConstantHeadingInterpolation(currentPose.getHeading())
                                .setZeroPowerAccelerationMultiplier(5)
                                .build();
                        adjustmentAction = new PathFollowingAction(follower, strafePath);
                    } else if (Math.abs(latestAngleError) > ANGLE_ERROR_THRESHOLD) {
                        telemetry.addData("Adjustment", "Turn for Angle_Error: %.2f", latestAngleError);
                        double turnAmountDegrees = latestAngleError * TURN_ADJUSTMENT_FACTOR;
                        adjustmentAction = new RelativeTurnDegreesAction(follower, Math.abs(turnAmountDegrees), turnAmountDegrees > 0);
                    }

                    if (adjustmentAction != null) {
                        currentExecutingAction = adjustmentAction;
                        run(currentExecutingAction);
                    } else { // Should be ALIGNED if no adjustment needed
                        currentState = AlignmentState.ALIGNED;
                        break; // break from switch to prevent immediate re-evaluation
                    }
                }
                // Check if the currently running adjustment action has finished
                if (currentExecutingAction != null && !runningActions.contains(currentExecutingAction)) {
                    alignmentAttempts++;
                    if (alignmentAttempts >= MAX_ALIGNMENT_ATTEMPTS) {
                        currentState = AlignmentState.FAILED;
                    } else {
                        currentState = AlignmentState.STARTING_ALIGNMENT; // Loop back to try again
                    }
                }
                break;

            case ALIGNED:
                telemetry.addLine("State: ALIGNED! Press A to re-align.");
                currentState = AlignmentState.IDLE;
                break;

            case FAILED:
                telemetry.addLine("State: FAILED after " + alignmentAttempts + " attempts. Press A to re-align.");
                currentState = AlignmentState.IDLE;
                break;
        }

        follower.update();
        motorControl.lift.setTargetPosition(235);
        motorControl.update();
        poseUpdater.update();

        if (poseUpdater != null) {
            Pose p = poseUpdater.getPose();
            if (!Double.isNaN(p.getX())) {
                Drawing.drawRobot(p, "#4CAF50");
                if (dashboardPoseTracker != null) {
                    dashboardPoseTracker.update();
                    Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
                }
                Drawing.sendPacket();
            }
        }
        telemetry.addData("Current State", currentState.toString());
        telemetry.addData("Alignment Attempts", alignmentAttempts);
        telemetry.addData("Follower Pose", follower.getPose().toString());
        telemetry.addData("LL H_Error", String.format("%.2f", latestHorizontalError));
        telemetry.addData("LL A_Error", String.format("%.2f", latestAngleError));
        telemetry.addData("LL Data Valid", limelightDataValid);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
        super.stop();
    }
}