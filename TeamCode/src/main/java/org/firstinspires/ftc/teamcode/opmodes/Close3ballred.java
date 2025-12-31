package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "rednxt3ball")
public class Close3ballred extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2;

    // --- Turret + goal constants (MIRRORED across x = 72) ---

    // Original target was x = 16 (144-128) → mirrored → x' = 144 - 16 = 128
    public static double targetX = 128.0;
    public static double targetY = 125.0;

    // Original goal zone x = 28 (144-116) → mirrored → x' = 144 - 28 = 116
    public static double goalZoneX = 116.0;
    public static double goalZoneY = 116.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 135;    // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.02;

    @Override
    public void init() {
        super.init();  // important so PathChainAutoOpMode can init its state

        pathTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        // --- Hardware map ---
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

        // Shooter motor direction
        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servo init (same as your 18-ball)
        hood1.setPosition(0.48);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);
        turret1.setPosition(0.175);
        turret2.setPosition(0.15);
        indexfront.setPosition(RobotActions.INDEX_FRONT_EXTENDED);
        indexback.setPosition(RobotActions.INDEX_BACK_RETRACTED);

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                indexfront, indexback,
                hardwareMap.voltageSensor.iterator().next()
        );
        run(actions.safeindexer());

        // --- MIRRORED START POSE ---
        // Original: (23.185, 125.920, 144°)
        // Mirrored across x = 72 → x' = 144 - 23.185 = 120.815
        // Heading mirrored: 180° - 144° = 36°
        follower.setStartingPose(
                new Pose(120.815, 125.920, Math.toRadians(36.0))
        );

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();  // important so task state is reset correctly
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        updateTurretFromPose();

        // keep shooter spun up (tune as needed)
        run(actions.holdShooterAtRPMclose(1170, 30));

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

    @Override
    protected void buildPathChains() {
        // === ORIGINAL (for reference) ===
        // Path1: (23.185, 125.920) -> (62.322, 134.003), heading 144° -> 180°
        // Path2: (62.322, 134.003) -> (61.897, 127.196), constant heading 180°
        //
        // === MIRRORED across x = 72 (x' = 144 - x, θ' = 180° - θ) ===
        //
        // Path1:
        //   (23.185, 125.920) → (120.815, 125.920)
        //   (62.322, 134.003) → (81.678, 134.003)
        //   144° → 36°
        //   180° → 0°
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.815, 125.920),
                                new Pose(81.678, 134.003)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(36.0),
                        Math.toRadians(0.0)
                )
                .build();

        // Path2:
        //   (62.322, 134.003) → (81.678, 134.003)  (must match Path1 end)
        //   (61.897, 127.196) → (82.103, 127.196)
        //   heading 180° → 0°
        path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(81.678, 134.003),
                                new Pose(82.103, 127.196)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0.0))
                // Shoot near end of path2
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Path1 with 20-second pause after it finishes
        addPath(path1, 20.0);

        // Path2 with short wait at the end for shots to clear
        PathChainTask path2Task = new PathChainTask(path2, 1.5);
        tasks.add(path2Task);
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
        // Not used in this auto
    }

    /**
     * Uses Pedro pose to:
     *  - compute angle to goal
     *  - compute distance to goal
     *  - set turret1/turret2 servo positions accordingly
     */
    private void updateTurretFromPose() {
        if (follower == null || turret1 == null || turret2 == null) return;

        // Current robot pose from Pedro (Pinpoint)
        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading(); // radians

        // Vector from robot to turret target
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        // Distance to goal (inches, since your field coords are in inches)
        double distanceToGoalInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Field angle to target (0 rad = +X, CCW positive)
        double angleToTargetField = Math.atan2(deltaY, deltaX);

        // Angle of target relative to robot heading (what turret must turn)
        double turretAngle = angleToTargetField - currentHeading;

        // Normalize to [-PI, PI]
        while (turretAngle > Math.PI)  turretAngle -= 2.0 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2.0 * Math.PI;

        // Convert to degrees and apply trim
        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;

        // Clamp to allowed turret range
        double clampedAngle = Math.max(-turretMaxAngle,
                Math.min(turretMaxAngle, turretAngleDegrees));

        // Map angle → servo position
        double servoPosition;
        if (clampedAngle >= 0) {
            // Positive angle = turn right
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            // Negative angle = turn left
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        // Clamp servo range
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // Apply backlash compensation to turret1, both shifted slightly like teleop
        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        // Optional: telemetry so you can see what's happening in auto
        telemetry.addData("Turret dist (in)", "%.1f", distanceToGoalInches);
        telemetry.addData("Turret angle (deg)", "%.1f", turretAngleDegrees);
        telemetry.addData("Turret servo", "%.3f", servoPosition);
    }
}
