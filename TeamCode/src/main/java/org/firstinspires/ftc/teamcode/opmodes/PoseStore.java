package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.localization.Pose;

public class PoseStore {
    public static Pose lastPose = null;

    public static void save(Pose pose) {
        if (pose != null) lastPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static boolean hasSaved() {
        return lastPose != null;
    }
}
