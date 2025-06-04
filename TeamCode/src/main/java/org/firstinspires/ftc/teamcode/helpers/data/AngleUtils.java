package org.firstinspires.ftc.teamcode.helpers.data;

public class AngleUtils {
    /**
     * Normalizes an angle to the range [-PI, PI) radians.
     * @param angleRadians The angle in radians.
     * @return The normalized angle in radians.
     */
    public static double normalizeRadians(double angleRadians) {
        double modifiedAngle = angleRadians % (2 * Math.PI);
        // Ensure the result is in [0, 2*PI) first, then shift to [-PI, PI)
        modifiedAngle = (modifiedAngle + 2 * Math.PI) % (2 * Math.PI);
        if (modifiedAngle > Math.PI) {
            modifiedAngle -= 2 * Math.PI;
        }
        return modifiedAngle; // Output is in [-PI, PI)
    }

    /**
     * Calculates the shortest signed angle difference between two angles.
     * The result will be in the range [-PI, PI).
     * @param targetRadians The target angle in radians.
     * @param currentRadians The current angle in radians.
     * @return The shortest angle difference in radians.
     */
    public static double shortestAngleDifference(double targetRadians, double currentRadians) {
        return normalizeRadians(targetRadians - currentRadians);
    }
}