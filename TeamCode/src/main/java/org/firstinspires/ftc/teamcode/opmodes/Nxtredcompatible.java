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

@Autonomous(name = "NXTclosecompatibleRED")
public class Nxtredcompatible extends PathChainAutoOpMode {

    private static final double FIELD_SIZE = 144.0;

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    // --- Turret + goal constants (RED / UN-MIRRORED) ---
    public static double targetX = 128.0;
    public static double targetY = 135.0;

    public static double goalZoneX = 116.0;
    public static double goalZoneY = 145.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 135;    // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.02;

    // --- Lead-aim config ---
    private static final double TURRET_LEAD_SWITCH_T = 0.97;

    // Paths that end with launch3faster(), and their predicted end poses
    private final Set<PathChain> shooterPaths = new HashSet<>();
    private final Map<PathChain, Pose> predictedEndPoseByPath = new HashMap<>();

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

        // Shooter motor direction
        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servo init
        hood1.setPosition(0.48);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);
        turret1.setPosition(0.175);
        turret2.setPosition(0.15);

        actions = new RobotActions(
                intakefront, intakeback,
                shootr, shootl,
                launchgate, reargate,
                turret1, turret2,
                hood1
        );

        // ---- Start pose (RED / ORIGINAL, UN-MIRRORED) ----
        // Original start: (112.250, 136.421, -90 deg)
        follower.setStartingPose(new Pose(
                112.250, 136.421,
                Math.toRadians(-90)
        ));

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

        // keep shooter spun up
        run(actions.holdShooterAtRPMclose(1100, 30));

        runTasks();

        double vR   = shootr.getVelocity();
        double vL   = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avg  = 0.5 * (rpmR + rpmL);

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("RPM R", "%.0f", rpmR);
        telemetry.addData("RPM L", "%.0f", rpmL);
        telemetry.addData("RPM avg", "%.0f", avg);
        telemetry.update();
    }

    // ---------------- PATHS (RED / ORIGINAL, UN-MIRRORED) ----------------

    @Override
    protected void buildPathChains() {

        // P1: SHOOT
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(112.250, 136.421),
                        new Pose(98.321,   88.080)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-90),
                        Math.toRadians(0)
                )
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        // P2: INTAKE
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(98.321, 88.080),
                        new Pose(94.634, 80.501),
                        new Pose(122.000, 82.754)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // P3: SHOOT
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(122.000, 82.754),
                        new Pose(89.104,  82.959)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        // P4: INTAKE
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(89.104, 82.959),
                        new Pose(84.188, 55.511),
                        new Pose(115.000, 59.812)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // P5: INTAKE
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(115.000, 59.812),
                        new Pose(128.000, 75.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(-90)
                )
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // P6: SHOOT
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(128.000, 75.000),
                        new Pose(88.899,  83.368)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-90),
                        Math.toRadians(0)
                )
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        // P7: INTAKE
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(88.899, 83.368),
                        new Pose(81.320, 29.701),
                        new Pose(124.541, 35.232)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // P8: SHOOT
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(124.541, 35.232),
                        new Pose(88.489,  82.754)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(-63)
                )
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        // P9: INTAKE
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88.489, 82.754),
                        new Pose(124.000, 74.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-63),
                        Math.toRadians(-90)
                )
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Register shooter paths for lead-aim (predicted end pose during first 90%)
        shooterPaths.clear();
        predictedEndPoseByPath.clear();

        registerShooterPath_Direct(path1, new Pose(98.321, 88.080, Math.toRadians(0)));
        registerShooterPath_Direct(path3, new Pose(89.104, 82.959, Math.toRadians(0)));
        registerShooterPath_Direct(path6, new Pose(88.899, 83.368, Math.toRadians(0)));
        registerShooterPath_Direct(path8, new Pose(88.489, 82.754, Math.toRadians(-63)));
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        addPath(path1, 0.7);
        addPath(path2, 0);
        addPath(path3, 0.7);
        addPath(path4, 0);
        addPath(path5, 0);
        addPath(path6, 0.7);
        addPath(path7, 0);
        addPath(path8, 0.7);
        addPath(path9, 0);
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
        follower.followPath((PathChain) task.pathChain, true); // holdEnd = true
    }

    @Override
    protected void startTurn(TurnTask task) {
        // Not used
    }

    // -----------------------------
    // Turret lead-aim implementation
    // -----------------------------

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private void registerShooterPath_Direct(PathChain path, Pose predictedEndPose) {
        shooterPaths.add(path);
        predictedEndPoseByPath.put(path, predictedEndPose);
    }

    private Pose getTurretAimPose() {
        if (follower == null) return null;

        Pose live = follower.getPose();

        // Only lead-aim while actively driving the current path
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
        double turretAngle = normalizeRadians(angleToTargetField - currentHeading);

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

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        telemetry.addData("Turret dist (in)", "%.1f", distanceToGoalInches);
        telemetry.addData("Turret angle (deg)", "%.1f", turretAngleDegrees);
        telemetry.addData("Turret servo", "%.3f", servoPosition);

        // Debug: confirm hard switch behavior
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
