package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;

public class PoseStore {
    // Legacy shared pose for older opmodes.
    public static Pose lastPose = null;

    // Alliance-specific handoff for auto -> teleop.
    public static Pose redPose = null;
    public static Pose bluePose = null;

    private static Pose copyOf(Pose pose) {
        return pose == null ? null : new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static void save(Pose pose) {
        lastPose = copyOf(pose);
    }

    public static void saveRed(Pose pose) {
        redPose = copyOf(pose);
        save(pose);
    }

    public static void saveBlue(Pose pose) {
        bluePose = copyOf(pose);
        save(pose);
    }

    public static boolean hasSaved() {
        return lastPose != null;
    }

    public static boolean hasSavedRed() {
        return redPose != null;
    }

    public static boolean hasSavedBlue() {
        return bluePose != null;
    }
}
