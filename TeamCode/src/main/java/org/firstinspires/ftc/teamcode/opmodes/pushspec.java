package org.firstinspires.ftc.teamcode.opmodes;

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
@Autonomous(name = "pushspec")
public class pushspec extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private Timer opModeTimer;  // additional timer if desired

    // -------- Poses --------



    private MotorControl.Limelight limelight;

    private boolean scan1Done, scan2Done = false;

    private boolean spitDone1, spitDone2, spitDone3 = false;

    // --- Vision turn related fields ---
    private TurnTask visionTurn1, visionTurn2;
    private Vector2d latestVisionPose = new Vector2d(0, 0);
    private double latestVisionAngle = 0;
    private double lastDistance = 0;



    private final AtomicBoolean specimenProcessingComplete = new AtomicBoolean(false);


    // -------- PathChains --------

    private boolean visionStarted     = false;
    private double  visionStartTime   = 0.0;  // in seconds
    private PathChain preload, gotopush, push1, push2, push3, gotointake, score, returntointake,score2, returntointake2,
    score3, returntointake3, score4, returntointake4;


    /**
     * Collects a single set of Limelight samples and stores the results
     * into {@code latestVisionPose} (horizontal, forward) and
     * {@code latestVisionAngle}. Returns true if successful.
     */


    // -------- Override buildPathChains() --------
    @Override
    protected void buildPathChains() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(8, 56),
                        new Point(42, 67)))
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .setConstantHeadingInterpolation(0)
                .build();

        gotopush = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(42, 67),
                        new Point(28, 60),
                        new Point(6, 42),
                        new Point(30, 40)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0, () -> run(motorActions.intakeSpecimen()))
                .build();

        push1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(30, 40),
                        new Point(45, 39),
                        new Point(102, 18),
                        new Point(13, 23)))
                .setConstantHeadingInterpolation(0)
                .build();

        push2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(13, 23),
                        new Point(89, 30),
                        new Point(66, 5),
                        new Point(13, 14)))
                .setConstantHeadingInterpolation(0)
                .build();

        push3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(13, 14),
                        new Point(75, 20),
                        new Point(88, 4),
                        new Point(12, 8)))
                .setConstantHeadingInterpolation(0)
                .build();

        gotointake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(12, 8),
                        new Point(33, 36),
                        new Point(8, 35)))
                .setConstantHeadingInterpolation(0)
                .build();

        score = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(1, 35),
                        new Point(27, 70),
                        new Point(42, 71)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        returntointake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(42, 71),
                        new Point(31, 73),
                        new Point(26, 34),
                        new Point(8, 35)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0, () -> run(motorActions.intakeSpecimen()))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(1, 35),
                        new Point(27, 70),
                        new Point(42, 71)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        returntointake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(42, 71),
                        new Point(31, 73),
                        new Point(26, 34),
                        new Point(8, 35)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0, () -> run(motorActions.intakeSpecimen()))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(1, 35),
                        new Point(27, 70),
                        new Point(42, 71)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        returntointake3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(42, 71),
                        new Point(31, 73),
                        new Point(26, 34),
                        new Point(8, 35)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0, () -> run(motorActions.intakeSpecimen()))
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(1, 35),
                        new Point(27, 70),
                        new Point(42, 71)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        returntointake4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(42, 71),
                        new Point(31, 73),
                        new Point(26, 34),
                        new Point(8, 35)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0, () -> run(motorActions.intakeSpecimen()))
                .build();
    }



    @Override
    protected void buildTaskList() {
        tasks.clear();

        addPath(preload, 0);
        addPath(gotopush, 0);
        addPath(push1, 0);
        addPath(push2, 0);
        addPath(push3, 0);
        addPath(gotointake, 0.3);
        addPath(score, 0);
        addPath(returntointake, 0.3);
        addPath(score2, 0);
        addPath(returntointake2, 0.3);
        addPath(score3, 0);
        addPath(returntointake3, 0.3);
        addPath(score4, 0);
        addPath(returntointake4, 0.3);

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
        follower.setStartingPose(new Pose(8, 56, Math.toRadians(0)));  // Assuming heading 0 degrees





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
        run(motorActions.outtakeSpecimen());
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
