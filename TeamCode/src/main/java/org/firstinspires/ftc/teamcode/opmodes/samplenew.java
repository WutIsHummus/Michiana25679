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


@Autonomous(name = "4+ sample!!")
public class samplenew extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private Timer opModeTimer;  // additional timer if desired

    // -------- Poses --------
    private final Pose startPose   = new Pose(9, 111, Math.toRadians(270));
    private final Pose scorePose   = new Pose(18, 130, Math.toRadians(340));

    private final Pose subscorepose   = new Pose(24, 132, Math.toRadians(340));
    private final Pose thirdgrab = new Pose(18.5, 130, Math.toRadians(25));

    // Park poses
    private final Pose parkPose        = new Pose(62, 93, Math.toRadians(270));

    private final Pose subintakepose        = new Pose(62, 97, Math.toRadians(270));
    private final Pose parkControlPose = new Pose(53, 120, Math.toRadians(270));
    private MotorControl.Limelight limelight;

    private boolean scan1Done, scan2Done = false;
    private boolean scan3Done = false;


    private boolean eatdone1, eatdone2, eatdone3, firstcycle, depodone1, depodone2 = false;

    // --- Vision turn related fields ---
    private TurnTask visionTurn1, visionTurn2, visionTurn3;

    private double latestVisionAngle = 0;
    private double lastDistance = 0;

    private boolean subscoreDone1 = false;
    private boolean subscoreDone2 = false;
    private boolean subscoreDone3 = false;


    private final AtomicBoolean specimenProcessingComplete = new AtomicBoolean(false);


    // -------- PathChains --------
    private PathChain scorePreload;
    private PathChain intake1, intake2, intake3;
    private PathChain score1, score2, score3;

    private PathChain subscore1, subscore2, subscore3;
    private PathChain parkChain1, parkChain2, parkChain3, thirdgrabpath;

    private PathChain parkToControl, controlToSubintake;


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

        thirdgrabpath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(thirdgrab)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdgrab.getHeading())
                .addParametricCallback(0.9, () -> run(motorActions.extendo.set(100)))
                .build();

        // buildPathChains(): split your park chain in two
        parkToControl = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), subintakepose.getHeading())
                .addParametricCallback(0.9, () -> run(motorActions.lift.vision()))
                .addParametricCallback(0.9,   () -> run(motorActions.sweeper.extended()))
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.inArm.sampleExtended(),
                        motorActions.inPivot.sampleExtended()
                )))
                .build();

        controlToSubintake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(parkPose),new Point(subintakepose)))
                .setConstantHeadingInterpolation(subintakepose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .addParametricCallback(1, () -> run(motorActions.sweeper.retracted()))
                .build();



        parkChain2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(subintakepose)
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(),subintakepose.getHeading())
                .addParametricCallback(0.9, () -> run(motorActions.lift.vision()))
                .addParametricCallback(0.2, () -> run(motorActions.outtakeTransfer()))
                .addParametricCallback(0, () -> run(motorActions.extendo.retracted()))
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.inArm.sampleExtended(),
                        motorActions.inPivot.sampleExtended()
                )))
                .build();


        parkChain3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(subintakepose)
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(),subintakepose.getHeading())
                .addParametricCallback(0.9, () -> run(motorActions.lift.vision()))
                .addParametricCallback(0.2, () -> run(motorActions.outtakeTransfer()))
                .addParametricCallback(0, () -> run(motorActions.extendo.retracted()))
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakeTransfer(),
                        motorActions.outtakeSampleAuto2(),
                        motorActions.extendo.set(150)
                )))
                .build();



        subscore1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakeTransferAuto(),
                        new ParallelAction(
                                motorActions.outtakeSampleAuto2(),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        motorActions.extendo.set(150)
                                )
                        )
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
                .build();

        subscore2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakeTransferAuto(),
                        new ParallelAction(
                                motorActions.outtakeSampleAuto2(),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        motorActions.extendo.set(150)
                                )
                        )
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
                .build();

        subscore3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakeTransferAuto(),
                        new ParallelAction(
                                motorActions.outtakeSampleAuto2(),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        motorActions.extendo.set(150)
                                )
                        )
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
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

        addTurnToDegrees(-2, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.spin.eat(),
                        motorActions.extendo.set(600),
                        new SleepAction(0.05),
                        motorActions.inArm.specimenGrab(),
                        motorActions.inPivot.specimenGrab(),
                        motorActions.spin.eat(),
                        motorActions.extendo.waitUntilFinished(),
                        new SleepAction(0.1),
                        telemetryPacket -> {
                            eatdone1 = true; return false;
                        },
                        motorActions.intakeTransfer(),
                        motorActions.extendo.waitUntilFinished()
                ))
                .setMaxWaitTime(3)
                .setWaitCondition(() -> eatdone1)
        ;

        addTurnToDegrees(340, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeSampleAuto(),
                        telemetryPacket -> {
                            depodone1 = true; return false;
                        }

                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> depodone1)
        ;

        addPath(thirdgrabpath,0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.spin.eat(),
                        new SleepAction(0.05),
                        motorActions.extendo.set(660),
                        motorActions.inArm.specimenGrab(),
                        motorActions.inPivot.specimenGrab(),
                        motorActions.extendo.waitUntilFinished(),
                        new SleepAction(0.1),
                        telemetryPacket -> {
                            eatdone2 = true; return false;
                        },
                        motorActions.intakeTransfer(),
                        motorActions.extendo.waitUntilFinished()
                ))
                .setMaxWaitTime(1)
                .setWaitCondition(() -> eatdone2);

        addTurnToDegrees(340, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeSampleAuto(),
                        motorActions.lift.waitUntilFinished(),
                        telemetryPacket -> {
                            depodone2 = true; return false;
                        }

                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> depodone2)
        ;

        addPath(parkToControl,  0);

        // Vision + parkChain1
        addPath(controlToSubintake, 0)
                .addWaitAction(0.5, new SequentialAction(
                        motorActions.lift.waitUntilFinished(),
                        telemetryPacket -> {
                            MotorControl.Limelight.DetectionResult dr = limelight.getDistance(3);
                            if (dr != null) {
                                latestVisionAngle = Math.min(60, Math.max(-dr.yawDegrees, -15));
                                double yawRad = Math.toRadians(dr.yawDegrees);
                                lastDistance = dr.distanceInches / Math.cos(yawRad);
                                scan1Done = true;
                                return false;
                            }
                            return true;
                        }
                ))
                .setMaxWaitTime(5)
                .setWaitCondition(() -> scan1Done);

        visionTurn1 = addRelativeTurnDegrees(0, true, 0)
                .setWaitCondition(()->!motorControl.isEmpty())
                .setMaxWaitTime(2)
                .addWaitAction(2.5, motorActions.intakeTransfer());

        addPath(subscore1, 0.2)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeTransfer(),
                        telemetryPacket -> {
                            subscoreDone1 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(0.2)
                .setWaitCondition(() -> subscoreDone1);

// Vision + parkChain2
        addPath(parkChain2, 0)
                .addWaitAction(0.5, new SequentialAction(
                        motorActions.lift.waitUntilFinished(),
                        telemetryPacket -> {
                            MotorControl.Limelight.DetectionResult dr = limelight.getDistance(3);
                            if (dr != null) {
                                latestVisionAngle = Math.min(60, Math.max(-dr.yawDegrees, -15));
                                double yawRad = Math.toRadians(dr.yawDegrees);
                                lastDistance = dr.distanceInches / Math.cos(yawRad);
                                scan2Done = true;
                                return false;
                            }
                            return true;
                        }
                ))
                .setMaxWaitTime(5)
                .setWaitCondition(() -> scan2Done);


        visionTurn2 = addRelativeTurnDegrees(0, true, 0)
                .setWaitCondition(()->!motorControl.isEmpty())
                .setMaxWaitTime(3)
                .addWaitAction(2.5, motorActions.intakeTransfer());

        addPath(subscore2, 0.2)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeTransfer(),
                        telemetryPacket -> {
                            subscoreDone2 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(0.2)
                .setWaitCondition(() -> subscoreDone2);

// Vision + parkChain3
        addPath(parkChain3, 0)
                .addWaitAction(0.5, new SequentialAction(
                            motorActions.lift.waitUntilFinished(),
                        telemetryPacket -> {
                            MotorControl.Limelight.DetectionResult dr = limelight.getDistance(3);
                            if (dr != null) {
                                latestVisionAngle = Math.min(60, Math.max(-dr.yawDegrees, -15));
                                double yawRad = Math.toRadians(dr.yawDegrees);
                                lastDistance = dr.distanceInches / Math.cos(yawRad);
                                scan3Done = true;
                                return false;
                            }
                            return true;
                        }
                ))
                .setMaxWaitTime(5)
                .setWaitCondition(() -> scan3Done);

        visionTurn3 = addRelativeTurnDegrees(0, true, 0)
                .setWaitCondition(()->!motorControl.isEmpty())
                .setMaxWaitTime(3)
                .addWaitAction(2.5, motorActions.intakeTransfer());

        addPath(subscore3, 0.2)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeTransfer(),
                        telemetryPacket -> {
                            subscoreDone3 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(1)
                .setWaitCondition(() -> subscoreDone3);


    }


    @Override
    protected boolean isPathActive() {
        BaseTask current = tasks.get(currentTaskIndex);
        if (current instanceof TurnTask) {
            return isTurning();
        }
        PathChainTask p = (PathChainTask) current;
        return p.initiated && follower.isBusy();
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

        limelight.setTargetClasses("yellow", "red");

        // Build the paths and tasks.
        buildPathChains();
        buildTaskList();
    }

    @Override
    protected void startTurn(TurnTask task) {
        if (task == visionTurn1 || task == visionTurn2 || task == visionTurn3) {



            if (lastDistance==0) return;

            if (lastDistance != 0) {
                task.addWaitAction(0, new ParallelAction(
                        motorActions.outtakeTransfer(),
                        new SequentialAction(
                                motorActions.sampleExtend(Math.min(lastDistance * 32.25, 800)),
                                motorActions.extendo.waitUntilFinished(),
                                motorActions.spin.eat(),
                                motorActions.spin.eatUntilStrict(Enums.DetectedColor.YELLOW, motorControl),
                                telemetryPacket -> {
                                    lastDistance=0;
                                    return false;
                                }
                        )
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
                .setPathEndHeadingConstraint(0.001)
                .setConstantHeadingInterpolation(targetHeadingRadians)
                .setZeroPowerAccelerationMultiplier(10)
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
        telemetry.addData("Task Name", tasks.get(currentTaskIndex).getClass().getSimpleName());
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
        telemetry.addData("Distance", lastDistance);

        telemetry.addData("LLDistance", limelight.getDistance(3));
        telemetry.update();
    }
}
