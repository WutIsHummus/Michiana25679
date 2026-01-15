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

@Autonomous(name = "1 - idiotblueautocompatible")
public class Nxtbluecompatible extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7,
            path8, path9, path10, path11, path12, path13;

    // =========================
    // BLUE turret target (mirror of RED target (128,125) -> (16,125))
    // =========================
    public static double targetX = 16.0;
    public static double targetY = 125.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 135.0;  // deg left/right from center

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.015;

    // Lead-aim switch: hold end-of-path angle until 0.85, then live tracking
    private static final double TURRET_LEAD_SWITCH_T = 0.8;

    // Shooter boost logic (same as your standard)
    private static final double SHOOT_RPM_BOOST = 1700.0;
    private static final double SHOOT_RPM_HOLD  = 1050.0;
    private static final double BOOST_EXIT_RPM  = 1050.0;
    private static final double BOOST_REARM_RPM = 850.0;
    private boolean shooterBoostActive = true;

    // Requested params
    private static final double SHOOT_CALLBACK_T = 0.98;
    private static final double SHOOT_PATH_DECEL = 0.47;
    private static final double NONSHOOT_DECEL   = 0.5;
    private static final double SHOOT_END_WAIT_S = 1.0;

    // Shooting paths set (explicit)
    private final Set<PathChain> shooterPaths = new HashSet<>();
    private final Map<PathChain, Pose> predictedEndPoseByPath = new HashMap<>();

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // Path classification (explicit, per your message)
    private static boolean isShootIdx(int i) {
        return i == 1 || i == 4 || i == 7 || i == 10 || i == 12;
    }
    private static boolean isGateIdx(int i) {
        return i == 3 || i == 6 || i == 9;
    }
    private static boolean isIntakeIdx(int i) {
        return i == 2 || i == 5 || i == 8 || i == 11;
    }
    private static boolean isParkIdx(int i) {
        return i == 13;
    }

    @Override
    public void init() {
        super.init();

        follower = Constants.createFollower(hardwareMap);

        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotor.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");

        hood1      = hardwareMap.get(Servo.class, "hood 1");
        turret1    = hardwareMap.get(Servo.class, "turret1");
        turret2    = hardwareMap.get(Servo.class, "turret2");
        reargate   = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");

        indexfront = hardwareMap.get(Servo.class, "indexfront");
        indexback  = hardwareMap.get(Servo.class, "indexback");

        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                indexfront, indexback,
                hardwareMap.voltageSensor.iterator().next()
        );

        // Required
        run(actions.safeindexer());

        // Start pose = first pose of Path1 (BLUE)
        follower.setStartingPose(new Pose(34.671, 134.428, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // Turret aiming
        updateTurretFromPose();
        hood1.setPosition(0.475);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);

        // Shooter RPM boost/hold
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
        telemetry.addData("RPM avg", "%.0f", avgRpm);
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {
        // -------------------------
        // BLUE PATHS
        // -------------------------

        // PATH 1 (SHOOT)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(34.671, 134.428),
                        new Pose(52.538, 84.869)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(270),
                        Math.toRadians(180)
                )
                .setGlobalDeceleration(SHOOT_PATH_DECEL)
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(actions.launch3faster()))
                .build();

        // PATH 2 (INTAKE)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(52.538, 84.869),
                        new Pose(22.0, 84.656)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(1)
                .setTValueConstraint(0.95)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 3 (GATE => INTAKE ONLY)
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(22.0, 84.656),
                        new Pose(24.886, 78.275),
                        new Pose(21.0, 74.021)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(90)
                )
                .setTValueConstraint(0.95)
                .setBrakingStrength(1)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 4 (SHOOT)
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(21.0, 74.021),
                        new Pose(52.538, 85.081)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(180)
                )
                .setGlobalDeceleration(SHOOT_PATH_DECEL)
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(actions.launch3faster()))
                .build();

        // PATH 5 (INTAKE)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(52.538, 85.081),
                        new Pose(46.369, 57.855),
                        new Pose(14.0, 58.706)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .setBrakingStrength(1)
                .setTValueConstraint(0.95)

                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 6 (GATE => INTAKE ONLY)
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(14.0, 58.706),
                        new Pose(19.569, 62.535),
                        new Pose(17.0, 66.789)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(270)
                )
                .setTValueConstraint(0.95)
                .setBrakingStrength(1)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 7 (SHOOT)
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(17.0, 66.789),
                        new Pose(52.538, 84.656)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(270),
                        Math.toRadians(230)
                )
                .setGlobalDeceleration(SHOOT_PATH_DECEL)
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(actions.launch3faster()))
                .build();

        // PATH 8 (INTAKE)
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(52.538, 84.656),
                        new Pose(51.900, 32.544),
                        new Pose(14.0, 33.394)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .setBrakingStrength(1)
                .setTValueConstraint(0.95)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 9 (GATE => INTAKE ONLY)
        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(14.0, 33.394),
                        new Pose(30.629, 59.557),
                        new Pose(17.0, 69.554)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(270)
                )
                .setBrakingStrength(1)
                .setTValueConstraint(0.95)

                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 10 (SHOOT)
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(17.0, 69.554),
                        new Pose(52.538, 84.869)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(270),
                        Math.toRadians(230)
                )
                .setGlobalDeceleration(SHOOT_PATH_DECEL)
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(actions.launch3faster()))
                .build();

        // PATH 11 (INTAKE)
        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(52.538, 84.869),
                        new Pose(52.538, 30.00),
                        new Pose(10.00, 30.00)
                ))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .setTValueConstraint(0.95)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 12 (SHOOT) (reversed(true) per your source note)
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.00, 30.00),
                        new Pose(52.325, 84.656)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTValueConstraint(0.95)
                .setGlobalDeceleration(SHOOT_PATH_DECEL)
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(actions.launch3faster()))
                .build();

        // PATH 13 (PARK)
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(52.325, 84.656),
                        new Pose(44.668, 75.722)
                ))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        // -------------------------
        // Register shooter paths for lead-aim prediction (end-of-path aim)
        // -------------------------
        registerShooterLine(path1,  34.671, 134.428, 52.538, 84.869, false);
        registerShooterLine(path4,  17.654, 74.021,  52.538, 85.081, false);
        registerShooterLine(path7,  16.591, 66.789,  52.538, 84.656, false);
        registerShooterLine(path10, 16.591, 69.554,  52.538, 84.869, false);

        // Path12 is a line and reversed(true)
        registerShooterLine(path12, 14.251, 31.480,  52.325, 84.656, true);
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // 1 SHOOT (wait 1.0)
        tasks.add(new PathChainTask(path1, SHOOT_END_WAIT_S));

        // 2 INTAKE
        addPath(path2, 0.0);

        // 3 GATE (intake-only)
        addPath(path3, 0.0);

        // 4 SHOOT (wait 1.0)
        tasks.add(new PathChainTask(path4, SHOOT_END_WAIT_S));

        // 5 INTAKE
        addPath(path5, 0.0);

        // 6 GATE
        addPath(path6, 0.0);

        // 7 SHOOT (wait 1.0)
        tasks.add(new PathChainTask(path7, SHOOT_END_WAIT_S));

        // 8 INTAKE
        addPath(path8, 0.0);

        // 9 GATE
        addPath(path9, 0.0);

        // 10 SHOOT (wait 1.0)
        tasks.add(new PathChainTask(path10, SHOOT_END_WAIT_S));

        // 11 INTAKE
        addPath(path11, 0.0);

        // 12 SHOOT (wait 1.0)
        tasks.add(new PathChainTask(path12, SHOOT_END_WAIT_S));

        // 13 PARK
        addPath(path13, 0.0);
    }

    @Override
    protected boolean isPathActive() { return follower.isBusy(); }

    @Override
    protected boolean isTurning() { return false; }

    @Override
    protected double getCurrentTValue() { return follower.getCurrentTValue(); }

    @Override
    protected void startPath(PathChainTask task) {
        follower.followPath((PathChain) task.pathChain, true);
    }

    @Override
    protected void startTurn(TurnTask task) {
        // Not used
    }

    // =========================
    // Lead-aim registration
    // =========================
    private static double headingFromPoints(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1, x2 - x1);
    }

    private void registerShooterLine(PathChain path,
                                     double xStart, double yStart,
                                     double xEnd, double yEnd,
                                     boolean reversed) {
        double tangent = headingFromPoints(xStart, yStart, xEnd, yEnd);
        double endHeading = reversed ? normalizeRadians(tangent + Math.PI) : normalizeRadians(tangent);
        shooterPaths.add(path);
        predictedEndPoseByPath.put(path, new Pose(xEnd, yEnd, endHeading));
    }

    private PathChain getActivePathIfAny() {
        if (currentTaskIndex < 0 || currentTaskIndex >= tasks.size()) return null;
        Object taskObj = tasks.get(currentTaskIndex);
        if (!(taskObj instanceof PathChainTask)) return null;
        PathChainTask pct = (PathChainTask) taskObj;
        if (!(pct.pathChain instanceof PathChain)) return null;
        return (PathChain) pct.pathChain;
    }

    private Pose getTurretAimPose() {
        Pose live = follower.getPose();

        // Only lead-aim while driving shooter paths
        if (taskPhase != 0) return live;

        PathChain active = getActivePathIfAny();
        if (active == null || !shooterPaths.contains(active)) return live;

        double t = follower.getCurrentTValue();
        if (t < TURRET_LEAD_SWITCH_T) {
            Pose predicted = predictedEndPoseByPath.get(active);
            if (predicted != null) return predicted;
        }

        return live;
    }

    // =========================
    // Turret aim (BLUE constraint)
    // =========================
    private void updateTurretFromPose() {
        if (turret1 == null || turret2 == null) return;

        Pose aimPose = getTurretAimPose();

        double currentX = aimPose.getX();
        double currentY = aimPose.getY();
        double currentHeading = aimPose.getHeading();

        double dx = targetX - currentX;
        double dy = targetY - currentY;

        double angleToTargetField = Math.atan2(dy, dx);
        double turretAngle = normalizeRadians(angleToTargetField - currentHeading);

        double turretAngleDeg = Math.toDegrees(turretAngle) + turretTrimDeg;
        double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDeg));

        double servoPosition;
        if (clampedAngle >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // BLUE constraint: only <= center
        servoPosition = Math.min(turretCenterPosition, servoPosition);

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));
        turret1Pos = Math.min(turretCenterPosition, turret1Pos);

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        double t = follower.getCurrentTValue();
        boolean predict = (taskPhase == 0 && t < TURRET_LEAD_SWITCH_T && shooterPaths.contains(getActivePathIfAny()));
        telemetry.addData("TurretAimMode", predict ? "PREDICT" : "LIVE");
        telemetry.addData("TurretAimT", "%.2f", t);
    }

    @Override
    public void stop() {
        try { PoseStore.save(follower.getPose()); } catch (Exception ignored) {}
        super.stop();
    }
}
