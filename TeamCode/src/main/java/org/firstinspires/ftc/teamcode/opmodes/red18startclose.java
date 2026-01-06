package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

@Autonomous(name = "red18startclose")
public class red18startclose extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, leave, path10, path11, path12;

    // --- Turret + goal constants (RED) ---
    public static double targetX = 128.0;
    public static double targetY = 125.0;

    public static double goalZoneX = 116.0;
    public static double goalZoneY = 116.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 135;    // deg left/right from center

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.015;

    // --- Lead-aim config ---
    private static final double TURRET_LEAD_SWITCH_T = 0.97;

    private final Set<PathChain> shooterPaths = new HashSet<>();
    private final Map<PathChain, Pose> predictedEndPoseByPath = new HashMap<>();

    // --- Shooter two-stage setpoint (spin-up boost then settle) ---
    private static final double SHOOT_RPM_BOOST = 1700;
    private static final double SHOOT_RPM_HOLD  = 1095.0;

    private static final double BOOST_EXIT_RPM  = 1100.0;
    private static final double BOOST_REARM_RPM = 850.0;

    private boolean shooterBoostActive = true;

    @Override
    public void init() {
        super.init();

        pathTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotor.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");
        hood1       = hardwareMap.get(Servo.class, "hood 1");
        turret1     = hardwareMap.get(Servo.class, "turret1");
        turret2     = hardwareMap.get(Servo.class, "turret2");
        reargate    = hardwareMap.get(Servo.class, "reargate");
        launchgate  = hardwareMap.get(Servo.class, "launchgate");
        indexfront  = hardwareMap.get(Servo.class, "indexfront");
        indexback   = hardwareMap.get(Servo.class, "indexback");

        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        hood1.setPosition(0.48);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);
        indexfront.setPosition(RobotActions.INDEX_FRONT_EXTENDED);
        indexback.setPosition(RobotActions.INDEX_BACK_RETRACTED);

        // Start turret on the allowed RED side (higher-than-center)
        turret1.setPosition(0.8);
        turret2.setPosition(0.8);

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                indexfront, indexback,
                hardwareMap.voltageSensor.iterator().next()
        );
        run(actions.safeindexer());

        // UPDATED START POSE (RED SIDE) - per your request
        follower.setStartingPose(new Pose(112.250, 136.421, Math.toRadians(-90)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        updateTurretFromPose();

        double vR   = shootr.getVelocity();
        double vL   = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avgRpm = 0.5 * (rpmR + rpmL);

        if (shooterBoostActive) {
            if (avgRpm >= BOOST_EXIT_RPM) shooterBoostActive = false;
        } else {
            if (avgRpm <= BOOST_REARM_RPM) shooterBoostActive = true;
        }

        double requestedRpm = shooterBoostActive ? SHOOT_RPM_BOOST : SHOOT_RPM_HOLD;
        run(actions.holdShooterAtRPMclose(requestedRpm, 30));

        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Mode", shooterBoostActive ? "BOOST" : "HOLD");
        telemetry.addData("Target RPM", "%.0f", requestedRpm);
        telemetry.addData("RPM R", "%.0f", rpmR);
        telemetry.addData("RPM L", "%.0f", rpmL);
        telemetry.addData("RPM avg", "%.0f", avgRpm);

        telemetry.update();
    }

    @Override
    protected void buildPathChains() {
        // =========================
        // MANUALLY MIRRORED PATHS
        // Mirror rule used: x' = 144 - x, y unchanged.
        // =========================

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(112.250, 136.431),
                        new Pose(87.421, 87.421)))
                .setConstantHeadingInterpolation(Math.toRadians(-70))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.85, () -> run(actions.launch3faster()))
                .build();

        leave= follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((87.421), 87.421),
                        new Pose((112), 87)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(87.421, 87.421),
                        new Pose(126.000, 81.000)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(124.000, 79.000),
                        new Pose(87.208, 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.208, 87.634),
                        new Pose(89.335, 31.000),
                        new Pose(132.000, 25.000)))
                .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(0))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(135.000, 25.000),
                        new Pose(89.335, 35.734),
                        new Pose(87.421, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(88.000, 87.000),
                        new Pose(104.012, 52),
                        new Pose(125.000, 57),
                        new Pose(134, 60)))
                .setTangentHeadingInterpolation()
                .setVelocityConstraint(20)
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(132.000, 58.000),
                        new Pose(120.000, 50.000),
                        new Pose(87.208, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.6, () -> run(actions.launch3faster()))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(90.824, 89.335),
                        new Pose(131.000, 62.109),
                        new Pose(135.000, 51.261),
                        new Pose(137.000, 12.000)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(132.514, 11.273),
                        new Pose(87.421, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.421, 87.634),
                        new Pose(74.000, 11.000),
                        new Pose(132.000, 8.721)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(124.000, 11.273),
                        new Pose(82.000, 99)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        registerShooterPath_Line(path1, 88.000, 8.000, 87.421, 87.421, true);
        registerShooterPath_Line(path3, 124.000, 79.000, 87.208, 87.421, true);

        registerShooterPath_CurveEnd(path5,
                89.335, 35.734,
                87.421, 87.634,
                87.421, 87.634,
                true);

        registerShooterPath_CurveEnd(path7,
                120.000, 50.000,
                87.208, 87.634,
                87.208, 87.634,
                true);

        registerShooterPath_Line(path9, 132.514, 11.273, 87.421, 87.634, true);
        registerShooterPath_Line(path12, 124.000, 11.273, 82.000, 97.000, true);
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        PathChainTask path1Task = new PathChainTask(path1, 0.7);
        tasks.add(path1Task);

        addPath(path2, 0);

        PathChainTask path3Task = new PathChainTask(path3, 0.7);
        tasks.add(path3Task);

        addPath(path4, 0);

        PathChainTask path5Task = new PathChainTask(path5, 0.7);
        tasks.add(path5Task);

        addPath(path6, 0);

        PathChainTask path7Task = new PathChainTask(path7, 0.7);
        tasks.add(path7Task);

        addPath(path8, 0);

        PathChainTask path9Task = new PathChainTask(path9, 0.7);
        tasks.add(path9Task);
        addPath(path8,0);
        addPath(path9,0.7);
        addPath(leave,1);
    }

    @Override
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected boolean isTurning() {
        return false;
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
    protected void startTurn(TurnTask task) {
        // Not used
    }

    private static double headingFromPoints(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1, x2 - x1);
    }

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private void registerShooterPath_Line(PathChain path, double xStart, double yStart,
                                          double xEnd, double yEnd, boolean reversed) {
        double tangent = headingFromPoints(xStart, yStart, xEnd, yEnd);
        double endHeading = reversed ? normalizeRadians(tangent + Math.PI) : normalizeRadians(tangent);
        shooterPaths.add(path);
        predictedEndPoseByPath.put(path, new Pose(xEnd, yEnd, endHeading));
    }

    private void registerShooterPath_CurveEnd(PathChain path,
                                              double xLastControl, double yLastControl,
                                              double xEnd, double yEnd,
                                              double poseXEnd, double poseYEnd,
                                              boolean reversed) {
        double tangent = headingFromPoints(xLastControl, yLastControl, xEnd, yEnd);
        double endHeading = reversed ? normalizeRadians(tangent + Math.PI) : normalizeRadians(tangent);
        shooterPaths.add(path);
        predictedEndPoseByPath.put(path, new Pose(poseXEnd, poseYEnd, endHeading));
    }

    private Pose getTurretAimPose() {
        if (follower == null) return null;

        Pose live = follower.getPose();

        if (taskPhase != 0) return live;
        if (currentTaskIndex < 0 || currentTaskIndex >= tasks.size()) return live;

        Object taskObj = tasks.get(currentTaskIndex);
        if (!(taskObj instanceof PathChainTask)) return live;

        PathChainTask pct = (PathChainTask) taskObj;
        if (!(pct.pathChain instanceof PathChain)) return live;

        PathChain activePath = (PathChain) pct.pathChain;

        if (!shooterPaths.contains(activePath)) return live;

        double t = follower.getCurrentTValue();
        if (t < TURRET_LEAD_SWITCH_T) {
            Pose predicted = predictedEndPoseByPath.get(activePath);
            if (predicted != null) return predicted;
        }

        return live;
    }

    private void updateTurretFromPose() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getTurretAimPose();
        if (aimPose == null) return;

        double currentX = aimPose.getX();
        double currentY = aimPose.getY();
        double currentHeading = aimPose.getHeading();

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        double distanceToGoalInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        double angleToTargetField = Math.atan2(deltaY, deltaX);

        double turretAngle = angleToTargetField - currentHeading;
        turretAngle = normalizeRadians(turretAngle);

        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;

        double clampedAngle = Math.max(-turretMaxAngle,
                Math.min(turretMaxAngle, turretAngleDegrees));

        double servoPosition;
        if (clampedAngle >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // RED SIDE: allow only the higher-than-center side
        servoPosition = Math.max(turretCenterPosition, servoPosition);

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1Pos = Math.max(turretCenterPosition, turret1Pos);

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        telemetry.addData("Turret dist (in)", "%.1f", distanceToGoalInches);
        telemetry.addData("Turret angle (deg)", "%.1f", turretAngleDegrees);
        telemetry.addData("Turret servo", "%.3f", servoPosition);

        double t = follower.getCurrentTValue();
        boolean predict = (taskPhase == 0 && t < TURRET_LEAD_SWITCH_T);
        telemetry.addData("TurretAimMode", predict ? "PREDICT" : "LIVE");
        telemetry.addData("TurretAimT", "%.2f", t);
    }

    @Override
    public void stop() {
        try {
            PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}
