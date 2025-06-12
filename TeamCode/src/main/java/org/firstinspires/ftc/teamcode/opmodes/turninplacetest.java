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
@Autonomous(name = "turninplacetest")
public class turninplacetest extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private Timer opModeTimer;  // additional timer if desired

    // -------- Poses --------
    private final Pose startPose = new Pose(9, 56, Math.toRadians(0));
    private final Pose preloadPose = new Pose(42, 70, Math.toRadians(0));
    private final Pose scorePose = new Pose(41,     68, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(41, 68, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(41, 68, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(41, 68, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(41, 68, Math.toRadians(0));
    private final Pose prescore = new Pose(38, 68, Math.toRadians(25));
    private final Pose pickup1Pose = new Pose(20, 24, Math.toRadians(0));

    private final Pose intake = new Pose(11, 35, Math.toRadians(0));
    private final Pose turninplacepose = new Pose(24, 35, Math.toRadians(230));
    private final Pose intakeControl3 = new Pose(30, 34, Math.toRadians(0));
    private final Pose parkPose = new Pose(11, 22, Math.toRadians(90));
    private final Pose parkControlPose = new Pose(12, 74, Math.toRadians(90));


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
    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup2, grabPickup3;
    private PathChain depositHP1, depositHP2, depositHP3;
    private PathChain intake1, intake2, intake3, intake4, intake5;
    private PathChain  score1, score2, score3, score4, score5;
    private PathChain  vision1deposit, vision2intake, vision2deposit;
    private PathChain  preoloadIntake;
    private PathChain parkChain;

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
        // 1. Preload score path.
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0, () -> run(motorActions.safeServos()))
                .setZeroPowerAccelerationMultiplier(3)
                .addParametricCallback(0.98, () -> run(new SequentialAction(motorActions.depositSpecimen())))
                .addParametricCallback(0.2, () -> run(motorActions.specimenExtend(0)))
                .build();

        // 2. Vision deposit from preload to intake.
        vision1deposit = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(preloadPose),
                        new Point(intake)))
                .addParametricCallback(0.2, () -> run(new SequentialAction(
                        motorActions.spitSample(),
                        motorActions.spin.waitUntilEmpty(motorControl),
                        motorActions.lift.transfer())))
                .addParametricCallback(0.85, () -> run(motorActions.spin.poop()))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), intake.getHeading())
                .build();

        // 3. Vision intake from intake to scorePose.
        vision2intake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(scorePose)))
                .addParametricCallback(0, () -> run(motorActions.spin.stop()))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        // 4. Vision deposit from scorePose to pickup1Pose.
        vision2deposit = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(turninplacepose)))
                .addParametricCallback(0, () -> run(motorActions.spin.stop()))
                .addParametricCallback(0, () -> run(motorActions.spitSample()))
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.extendo.set(180))))
                .addParametricCallback(0.90, () -> run(motorActions.spin.poop()))
                .setLinearHeadingInterpolation(Math.toRadians(pickup1Pose.getHeading()), Math.toRadians(turninplacepose.getHeading()))
                .build();

        // 5. Intake 1: pickup1 → intake
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(turninplacepose),
                        new Point(intake)))
                .setConstantHeadingInterpolation(Math.toRadians(intake.getHeading()))
                .addParametricCallback(0.5, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0, () -> run(motorActions.intakeSpecimen()))
                .build();

        // 6. Score 1: intake → scorePose1
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(prescore),
                        new Point(scorePose1)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0, () -> motorControl.spin.setPower(0))
                .build();

        // 7. Intake 2: scorePose → intake
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intake)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake.getHeading(), 50)
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0.2, () -> run(motorActions.intakeSpecimen()))
                .build();

        // 8. Score 2
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(prescore),
                        new Point(scorePose2)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        // 9. Intake 3
        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intake)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake.getHeading(), 50)
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0.2, () -> run(motorActions.intakeSpecimen()))
                .build();

        // 10. Score 3
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(prescore),
                        new Point(scorePose3)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        // 11. Intake 4
        intake4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intake)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake.getHeading(), 50)
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0.2, () -> run(motorActions.intakeSpecimen()))
                .build();

        // 12. Score 4
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(prescore),
                        new Point(scorePose4)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        // 13. Intake 5
        intake5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intake)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake.getHeading(), 50)
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0.2, () -> run(motorActions.intakeSpecimen()))
                .build();

        // 14. Score 5
        score5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(prescore),
                        new Point(scorePose4)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        // 15. Park path (used post-scoring).
        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose4),
                        new Point(parkControlPose),
                        new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();
    }


    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Preload task.
        addPath(scorePreload, 0);

        // Vision-based turn before first vision deposit

        addPath(vision1deposit, 0.2).addWaitAction(0,motorActions.outtakeSpecimen());




        addPath(vision2intake, 1).addWaitAction(0,
                new SequentialAction(
                        motorActions.depositSpecimen(),
                        motorActions.lift.specimen()
                ));

        // Vision-based turn before second vision deposit

        addPath(vision2deposit, 0);

        addTurnToDegrees(320, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.set(358),
                        motorActions.grabUntilSpecimen(),
                        telemetryPacket -> {
                            spitDone1 = true; return false;
                        }
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone1)
        ;

        addTurnToDegrees(240, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.set(358),
                        motorActions.grabUntilSpecimen(),
                        telemetryPacket -> {
                            spitDone1 = true; return false;
                        }
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone2)
        ;

        addTurnToDegrees(310, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.set(480),
                        motorActions.grabUntilSpecimen(),
                        telemetryPacket -> {
                            spitDone2 = true; return false;
                        }
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone2)
        ;

        addTurnToDegrees(240, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.set(480),
                        motorActions.grabUntilSpecimen(),
                        telemetryPacket -> {
                            spitDone2 = true; return false;
                        }
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone2)
        ;

        addTurnToDegrees(300, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.set(730),
                        motorActions.grabUntilSpecimen(),
                        motorActions.extendo.set(600),
                        telemetryPacket -> {
                            spitDone2 = true; return false;
                        }
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone2)
        ;

        addTurnToDegrees(240, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.set(600),
                        motorActions.grabUntilSpecimen(),
                        telemetryPacket -> {
                            spitDone2 = true; return false;
                        }
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone2)
        ;





        addPath(intake1, 0.1);


        // Score task 1.
        tasks.add(new PathChainTask(score1, 0));

        // Intake task 2.
        tasks.add(new PathChainTask(intake2, 0));

        // Score task 2.
        tasks.add(new PathChainTask(score2, 0));

        // Intake task 3.
        tasks.add(new PathChainTask(intake3, 0));
        // Score task 3.
        tasks.add(new PathChainTask(score3, 0));

        // Intake task 4.
        tasks.add(new PathChainTask(intake4, 0));

        // Score task 4.
        tasks.add(new PathChainTask(score4, 0));

        tasks.add(new PathChainTask(intake5, 0));

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
