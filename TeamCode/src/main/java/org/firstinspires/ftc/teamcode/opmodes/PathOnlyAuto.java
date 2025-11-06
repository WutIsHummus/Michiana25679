package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Path Only Auto")
public class PathOnlyAuto extends OpMode {

    private Follower follower;
    private Path currentPath;
    private int pathIndex = 0;
    private Path[] paths;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        // Start Point from the canvas: (24.460856, 125.920236) with heading ~144°
        follower.setStartingPose(new Pose(24.460856, 125.920236, Math.toRadians(144)));

        // Build all paths
        buildPaths();

        telemetry.addLine("Path Only Auto Initialized");
        telemetry.addLine("=================================");
        telemetry.addData("Total Paths", paths.length);
        telemetry.addLine("Just follows the path - no actions");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start following first path
        pathIndex = 0;
        follower.followPath(paths[pathIndex], true);
    }

    @Override
    public void loop() {
        follower.update();

        // When path is complete, move to next one
        if (!follower.isBusy() && pathIndex < paths.length - 1) {
            pathIndex++;
            follower.followPath(paths[pathIndex], true);
        }

        // Telemetry
        Pose currentPose = follower.getPose();
        telemetry.addData("Current Path", (pathIndex + 1) + "/" + paths.length);
        telemetry.addData("Position", String.format("(%.1f, %.1f)", currentPose.getX(), currentPose.getY()));
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));
        telemetry.addData("Path Active", follower.isBusy());
        telemetry.update();
    }

    private void buildPaths() {
        // Path 1: (24.460856,125.920236) -> (49,96)  | heading 144° -> 134°
        Path path1 = new Path(new BezierLine(
                new Pose(24.460856, 125.920236, Math.toRadians(144)),
                new Pose(49.0, 96.0, Math.toRadians(134))));
        path1.setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(134));

        // Path 2: (49,96) -> (16,84) control (81.6779913,79.9763863) | 142° -> 180°
        Path path2 = new Path(new BezierCurve(
                new Pose(49.0, 96.0, Math.toRadians(142)),
                new Pose(81.6779913, 79.9763863),
                new Pose(16.0, 84.0, Math.toRadians(180))));
        path2.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        // Path 3: (16,84) -> (49,96) | 180° -> 142°
        Path path3 = new Path(new BezierLine(
                new Pose(16.0, 84.0, Math.toRadians(180)),
                new Pose(49.0, 96.0, Math.toRadians(142))));
        path3.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142));

        // Path 4: (49,96) -> (17,60) control (76.57311669,53.3884786) | 142° -> 180°
        Path path4 = new Path(new BezierCurve(
                new Pose(49.0, 96.0, Math.toRadians(142)),
                new Pose(76.57311669, 53.3884786),
                new Pose(17.0, 60.0, Math.toRadians(180))));
        path4.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        // Path 5: (17,60) -> (14.0384047,72.10635156) control (73,75) | 180° -> 90°
        Path path5 = new Path(new BezierCurve(
                new Pose(17.0, 60.0, Math.toRadians(180)),
                new Pose(73.0, 75.0),
                new Pose(14.0384047, 72.10635156, Math.toRadians(90))));
        path5.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90));

        // Path 6: (14.0384,72.1063) -> (49,96) | 90° -> 142°
        Path path6 = new Path(new BezierLine(
                new Pose(14.0384047, 72.10635156, Math.toRadians(90)),
                new Pose(49.0, 96.0, Math.toRadians(142))));
        path6.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(142));

        // Path 7: (49,96) -> (14.88921713,35.9468424) control (92.9512555,31.0546528) | 142° -> 180°
        Path path7 = new Path(new BezierCurve(
                new Pose(49.0, 96.0, Math.toRadians(142)),
                new Pose(92.9512555, 31.0546528),
                new Pose(14.88921713, 35.9468424, Math.toRadians(180))));
        path7.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        // Path 8: (14.8892,35.9468) -> (48.7090103,95.9290989) | 180° -> 142°
        Path path8 = new Path(new BezierLine(
                new Pose(14.88921713, 35.9468424, Math.toRadians(180)),
                new Pose(48.7090103, 95.9290989, Math.toRadians(142))));
        path8.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142));

        // Path 9: (48.7090,95.9291) -> (10.42245195,10.2097488) control (14.25110782,54.664897) | 142° -> 270°
        Path path9 = new Path(new BezierCurve(
                new Pose(48.7090103, 95.9290989, Math.toRadians(142)),
                new Pose(14.25110782, 54.664897),
                new Pose(10.42245195, 10.2097488, Math.toRadians(270))));
        path9.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(270));

        // Path 10: (10.42245,10.20975) -> (48.9217344,95.9290898) | 270° -> 142°
        Path path10 = new Path(new BezierLine(
                new Pose(10.42245195, 10.2097488, Math.toRadians(270)),
                new Pose(48.9217344, 95.9290898, Math.toRadians(142))));
        path10.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(142));

        // Store all paths in array
        paths = new Path[]{path1, path2, path3, path4, path5, path6, path7, path8, path9, path10};
    }

    @Override
    public void stop() {
        // Stop the follower
    }
}

