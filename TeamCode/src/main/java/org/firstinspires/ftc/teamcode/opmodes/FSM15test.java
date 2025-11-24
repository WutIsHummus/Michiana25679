package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "FSM15test")
public class FSM15test extends PathChainAutoOpMode {

    private Follower follower;

    // Hardware
    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;

    private RobotActions actions;

    // Paths
    private PathChain path1, path2, path3, path4, path5, path6,
            path7, path8, path9, path10, path11, path12;

    // Pedro FSM
    private int pathState = 0;
    private boolean shotActionStarted = false;
    private static final double WAIT_SECONDS = 1.0;

    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(270));

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();   // pathTimer is from PathChainAutoOpMode
        shotActionStarted = false;
    }

    @Override
    public void init() {
        pathTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        // Hardware init
        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotor.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");
        hood1       = hardwareMap.get(Servo.class, "hood 1");
        turret1     = hardwareMap.get(Servo.class, "turret1");
        turret2     = hardwareMap.get(Servo.class, "turret2");
        reargate    = hardwareMap.get(Servo.class, "reargate");
        launchgate  = hardwareMap.get(Servo.class, "launchgate");

        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servo init
        hood1.setPosition(0.48);
        turret1.setPosition(0.275);
        turret2.setPosition(0.25);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.5);

        actions = new RobotActions(intakefront, intakeback, shootr, shootl,
                launchgate, reargate, turret1, turret2, hood1);

        buildPathChains();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        // Let ActionOpMode handle running any queued Actions
        super.loop();

        // Pedro follower update
        follower.update();

        // Keep shooter spun up (same as your original)
        run(actions.holdShooterAtRPMclose(1350, 30));

        // FSM path & shooting logic
        autonomousPathUpdate();

        // Telemetry
        double vR   = shootr.getVelocity();
        double vL   = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avg  = 0.5 * (rpmR + rpmL);

        Pose p = follower.getPose();

        telemetry.addData("pathState", pathState);
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("H (deg)", "%.1f", Math.toDegrees(p.getHeading()));
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("RPM R", "%.0f", rpmR);
        telemetry.addData("RPM L", "%.0f", rpmL);
        telemetry.addData("RPM avg", "%.0f", avg);
        telemetry.addData("waitTimer", "%.2f", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    // ===================== PATH BUILDING =====================

    @Override
    protected void buildPathChains() {
        // Path1: (56,8) -> (56.579,87.421), heading 270 -> 230
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(56.579, 87.421)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(230))
                .build();

        // Path2: (56.579,87.421) -> (20,82), heading 230 -> 180
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.421),
                        new Pose(20, 82)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .build();

        // Path3: (20,82) -> (56.792,87.421), heading 180 -> 230
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20, 82),
                        new Pose(56.792, 87.421)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                .build();

        // Path4: (56.792,87.421) -> (14,52) via (41.903,58.919), heading 230 -> 150
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.792, 87.421),
                        new Pose(41.903, 58.919),
                        new Pose(14, 52)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(150))
                .build();

        // Path5: (14,52) -> (56.792,87.634), heading 150 -> 230
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(14, 52),
                        new Pose(24, 50),
                        new Pose(56.792, 87.634)))
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(230))
                .build();

        // Path6: (56.792,87.634) -> (10,28) via (54.665,35.734), heading 230 -> 180
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.792, 87.634),
                        new Pose(54.665, 35.734),
                        new Pose(10, 28)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .build();

        // Path7: (10,28) -> (56.579,87.634), heading 180 -> 230
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10, 28),
                        new Pose(56.579, 87.634)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                .build();

        // Path8: (56.579,87.634) -> (11.486,11.273), heading 230 -> 250
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.634),
                        new Pose(11.486, 11.273)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(250))
                .build();

        // Path9: (11.486,11.273) -> (56.579,87.634), heading 250 -> 230
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(11.486, 11.273),
                        new Pose(56.579, 87.634)))
                .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(230))
                .build();

        // Path11: repeat hub -> stack
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.634),
                        new Pose(11.486, 11.273)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(250))
                .build();

        // Path12: repeat stack -> hub
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(11.486, 11.273),
                        new Pose(56.579, 87.634)))
                .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(230))
                .build();

        // Path10: final park
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.634),
                        new Pose(46.157, 76.360)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(230))
                .build();
    }

    // ===================== FSM LOGIC =====================

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start path1 (preload) with holdEnd
                follower.followPath(path1, false);
                setPathState(1);
                break;

            case 1:
                // Wait for path1 to finish
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                // 1s wait at hub after path1 + shoot
                if (!shotActionStarted) {
                    run(actions.launch3faster());
                    shotActionStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > WAIT_SECONDS) {
                    follower.followPath(path2, false);
                    setPathState(3);
                }
                break;

            case 3:
                // path2 -> intake lane
                if (!follower.isBusy()) {
                    follower.followPath(path3, false);
                    setPathState(4);
                }
                break;

            case 4:
                // wait for path3 (back to hub)
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                // 1s wait at hub after path3 + shoot
                if (!shotActionStarted) {
                    run(actions.launch3faster());
                    shotActionStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > WAIT_SECONDS) {
                    follower.followPath(path4, false);
                    setPathState(6);
                }
                break;

            case 6:
                // path4 -> out to second intake
                if (!follower.isBusy()) {
                    follower.followPath(path5, false);
                    setPathState(7);
                }
                break;

            case 7:
                // wait for path5 (back to hub)
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;

            case 8:
                // 1s wait at hub after path5 + shoot
                if (!shotActionStarted) {
                    run(actions.launch3faster());
                    shotActionStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > WAIT_SECONDS) {
                    follower.followPath(path6, false);
                    setPathState(9);
                }
                break;

            case 9:
                // path6 -> out to low intake
                if (!follower.isBusy()) {
                    follower.followPath(path7, false);
                    setPathState(10);
                }
                break;

            case 10:
                // wait for path7 (back to hub)
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11:
                // 1s wait at hub after path7 + shoot
                if (!shotActionStarted) {
                    run(actions.launch3faster());
                    shotActionStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > WAIT_SECONDS) {
                    follower.followPath(path8, false);
                    setPathState(12);
                }
                break;

            case 12:
                // path8 -> stack
                if (!follower.isBusy()) {
                    follower.followPath(path9, false);
                    setPathState(13);
                }
                break;

            case 13:
                // wait for path9 (stack -> hub)
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;

            case 14:
                // 1s wait at hub after path9 + shoot
                if (!shotActionStarted) {
                    run(actions.launch3faster());
                    shotActionStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > WAIT_SECONDS) {
                    follower.followPath(path11, false);
                    setPathState(15);
                }
                break;

            case 15:
                // path11 hub -> stack again
                if (!follower.isBusy()) {
                    follower.followPath(path12, false);
                    setPathState(16);
                }
                break;

            case 16:
                // wait for path12 (stack -> hub)
                if (!follower.isBusy()) {
                    setPathState(17);
                }
                break;

            case 17:
                // 1s wait at hub after path12 + shoot
                if (!shotActionStarted) {
                    run(actions.launch3faster());
                    shotActionStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > WAIT_SECONDS) {
                    follower.followPath(path10, false);
                    setPathState(18);
                }
                break;

            case 18:
                // final park path10
                if (!follower.isBusy()) {
                    setPathState(-1);  // done
                }
                break;

            case -1:
            default:
                // Idle
                break;
        }
    }

    // ===================== REQUIRED ABSTRACTS (NOT USED) =====================

    @Override
    protected void buildTaskList() {
        // We are NOT using the task system here
        tasks.clear();
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
        // Not used in FSM version
    }

    @Override
    protected void startTurn(TurnTask task) {
        // Not used in this auto
    }

    @Override
    public void stop() {
        try {
            org.firstinspires.ftc.teamcode.opmodes.PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}
