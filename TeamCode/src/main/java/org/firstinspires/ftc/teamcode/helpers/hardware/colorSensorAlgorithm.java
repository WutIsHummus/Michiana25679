package org.firstinspires.ftc.teamcode.helpers.hardware; // Assuming this is the correct package

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA; // Required for normalized colors

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.data.Enums; // Assuming your Enums class is here

import android.graphics.Color; // Required for HSV conversion

public class colorSensorAlgorithm {

    public RevColorSensorV3 internalColorSensor;

    // --- Existing RGB-based thresholds and methods ---
    public final double RED_THRESHOLD_LEFT = 700, RED_THRESHOLD_RIGHT = 700;
    public final double BLUE_THRESHOLD_LEFT = 700, BLUE_THRESHOLD_RIGHT = 700;
    public final double RED_THRESHOLD = 550;
    public final double BLUE_THRESHOLD = 550;
    public final double YELLOW_RED_VALUE = 0.419;
    public final double YELLOW_GREEN_VALUE = 0.3675;
    public final double YELLOW_BLUE_VALUE = 0.2116;
    public final double[] YELLOW_CONSTANTS = {YELLOW_RED_VALUE, YELLOW_GREEN_VALUE, YELLOW_BLUE_VALUE};
    public final double YELLOW_THRESHOLD = 0.12;
    public final double WHITE_RED_VALUE = 0.30125;
    public final double WHITE_GREEN_VALUE = 0.37125;
    public final double WHITE_BLUE_VALUE = 0.325;
    public final double[] WHITE_CONSTANTS = {WHITE_RED_VALUE, WHITE_GREEN_VALUE, WHITE_BLUE_VALUE};
    public final double WHITE_THRESHOLD = 0.01;
    public final double WHITE_TOTAL_COUNT = 800;
    public final double BLACK_ALPHA_VALUE = 325;

    public colorSensorAlgorithm(HardwareMap hardwareMap, String name){
        this.internalColorSensor = hardwareMap.get(RevColorSensorV3.class, name);
    }

    public double distance(){
        if (internalColorSensor != null) {
            return ((DistanceSensor) internalColorSensor).getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
        }
        return Double.NaN; // Or some other indicator that distance is not available
    }

    public int red(){ return internalColorSensor.red(); }
    public int green(){ return internalColorSensor.green(); }
    public int blue(){ return internalColorSensor.blue(); }
    public double total(){ return red() + green() + blue(); }

    public double[] rgb(){
        double[] arr = new double[3];
        arr[0] = red(); arr[1] = green(); arr[2] = blue();
        return arr;
    }

    public double[] normalizedRGB(){ // Your custom normalization
        double[] arr = new double[3];
        double[] originalArr = rgb();
        double total = 0;
        for(double i : originalArr) total+= i;
        if (total == 0) return new double[]{0,0,0}; // Avoid division by zero
        for(int i = 0; i < 3; i++){ arr[i] = originalArr[i] / total; }
        return arr;
    }

    public double arrayError(double[] arr1, double[] arr2){
        double total = 0;
        for(int i = 0; i < arr1.length; i++){ total += Math.pow(arr1[i] - arr2[i], 2); }
        return Math.sqrt(total);
    }

    public boolean isBlack(){ return (internalColorSensor.alpha() < BLACK_ALPHA_VALUE); }
    public int alphaValue(){ return internalColorSensor.alpha(); }
    public boolean isRed(){ return internalColorSensor.red() > RED_THRESHOLD; }
    public boolean isBlue(){ return internalColorSensor.blue() > BLUE_THRESHOLD; }
    public double yellowError(){ return arrayError(normalizedRGB(), YELLOW_CONSTANTS); }
    public double whiteError(){ return arrayError(normalizedRGB(), WHITE_CONSTANTS); }
    public boolean isYellow(){
        return yellowError() < YELLOW_THRESHOLD || (distance() > 6 && whiteError() > 0.02);
    }
    public boolean isWhite(){
        return whiteError() < WHITE_THRESHOLD && total() > WHITE_TOTAL_COUNT;
    }
    public String normalizedValues() {
        double[] norm = normalizedRGB();
        return String.format(java.util.Locale.US, "Custom Norm RGB: %.2f %.2f %.2f", norm[0], norm[1], norm[2]);
    }
    public void enableLED(boolean LEDMode){ internalColorSensor.enableLed(LEDMode); }
    public boolean withinColorRange(){ return isYellow() || isWhite(); }


    // --- NEW HSV-based Color Detection Method ---
    /**
     * Detects color based on HSV values.
     * IMPORTANT: Tune the HSV thresholds below by observing telemetry from a test OpMode.
     * @return The detected color as an Enums.DetectedColor.
     */
    public Enums.DetectedColor getDetectedColorHSV() {
        // Use the sensor's built-in normalized colors for more stable HSV conversion
        NormalizedRGBA normalizedColors = internalColorSensor.getNormalizedColors();

        float[] hsvValues = new float[3];
        // Color.RGBToHSV expects int 0-255, normalizedColors are float 0-1
        Color.RGBToHSV(
                (int) (normalizedColors.red * 255),
                (int) (normalizedColors.green * 255),
                (int) (normalizedColors.blue * 255),
                hsvValues
        );

        float hue = hsvValues[0];        // Hue: 0 to 360
        float saturation = hsvValues[1]; // Saturation: 0.0 to 1.0
        float value = hsvValues[2];      // Value (Brightness): 0.0 to 1.0

        // --- Define HSV Thresholds (STARTING POINTS - MUST BE TUNED) ---
        float minSaturationForColor = 0.30f;
        float minValueForColor = 0.15f;
        float maxValueForBlack = 0.12f;
        float maxSaturationForWhite = 0.25f;
        float minValueForWhite = 0.75f;

        // Hue ranges (0-360 degrees)
        float redHueMin1 = 0f, redHueMax1 = 25f;
        float redHueMin2 = 335f, redHueMax2 = 360f;
        float yellowHueMin = 40f, yellowHueMax = 75f; // Based on your data, yellow might be closer to 60-90 for hue
        // Raw RGB for Yellow (573, 845, 314) -> Norm (approx 0.31, 0.46, 0.17) -> HSV Hue ~50-60
        float blueHueMin = 180f, blueHueMax = 270f;  // Raw RGB for Blue (173, 347, 492) -> Norm (approx 0.17, 0.34, 0.48) -> HSV Hue ~200-220

        // --- Classification Logic ---
        if (value < maxValueForBlack && saturation < minSaturationForColor + 0.1) {
            return Enums.DetectedColor.BLACK;
        } else if (saturation < maxSaturationForWhite && value > minValueForWhite) {
            return Enums.DetectedColor.WHITE;
        } else if (saturation < minSaturationForColor && value > maxValueForBlack) {
            return Enums.DetectedColor.GRAY;
        } else if (saturation > minSaturationForColor && value > minValueForColor) {
            // Sufficiently colorful and bright to be a color
            if ((hue >= redHueMin1 && hue <= redHueMax1) || (hue >= redHueMin2 && hue <= redHueMax2)) {
                return Enums.DetectedColor.RED;
            } else if (hue >= yellowHueMin && hue <= yellowHueMax) {
                return Enums.DetectedColor.YELLOW;
            } else if (hue >= blueHueMin && hue <= blueHueMax) {
                return Enums.DetectedColor.BLUE;
            }
            // Add more colors (GREEN, CYAN, MAGENTA) here if needed
        }
        return Enums.DetectedColor.UNKNOWN;
    }

    /**
     * Provides direct HSV values for tuning.
     * @return float array [hue, saturation, value]
     */
    public float[] getHSVValues() {
        NormalizedRGBA normalizedColors = internalColorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.RGBToHSV(
                (int) (normalizedColors.red * 255),
                (int) (normalizedColors.green * 255),
                (int) (normalizedColors.blue * 255),
                hsvValues
        );
        return hsvValues;
    }
}
