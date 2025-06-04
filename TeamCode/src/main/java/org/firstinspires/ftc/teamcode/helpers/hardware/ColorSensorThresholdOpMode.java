package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.data.Enums;

@TeleOp(name = "Color Sensor Threshold", group = "Test")
public class ColorSensorThresholdOpMode extends OpMode {

    private RevColorSensorV3 sensor;

    @Override
    public void init() {
        // "color" must match your robot config name
        sensor = hardwareMap.get(RevColorSensorV3.class, "color");
        sensor.enableLed(true);

        telemetry.addData("Status", "Init complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        double distanceCm = getDistanceCm();
        double[] norm = normalizedRGB();

        // Compute scaled values: D * normalized channels
        double scaledR = distanceCm * norm[0];
        double scaledG = distanceCm * norm[1];
        double scaledB = distanceCm * norm[2];



        Enums.DetectedColor detected = Enums.DetectedColor.UNKNOWN;

        // Red thresholds:
        //   1.0  < D*Rnorm < 1.3
        //   0.9  < D*Gnorm < 1.7
        //   0.676< D*Bnorm < 1.45
        if (scaledR > 0.97  &&
                scaledG > 0.9  && scaledG < 1.4 &&
             scaledB < 1.2) {
            detected = Enums.DetectedColor.RED;
        }
        // Yellow thresholds:
        //   0.8  < D*Rnorm < 1.25
        //   1.2  < D*Gnorm < 1.9
        //   0.4  < D*Bnorm < 1.2
        else if (scaledR < 0.97 &&
                  scaledG > 1.2 && scaledG < 1.6  &&
                scaledB > 0.4  && scaledB < 1.2) {
            detected = Enums.DetectedColor.YELLOW;
        }
        // Blue thresholds:
        //   0.5  < D*Rnorm < 1.0
        //   0.95 < D*Gnorm < 1.9
        //   1.45 < D*Bnorm < 2.0
        else if (scaledR > 0.5   && scaledR < 1.05  &&
                scaledG > 0.95  && scaledG < 1.94  &&
                scaledB > 1.43 ) {
            detected = Enums.DetectedColor.BLUE;
        }

        if (distanceCm > 6){
            detected = Enums.DetectedColor.UNKNOWN;
        }

        // Send telemetry
        telemetry.addData("Distance (cm)", "%.2f", distanceCm);
        telemetry.addData("Norm RGB",    "%.2f, %.2f, %.2f", norm[0], norm[1], norm[2]);
        telemetry.addData("Scaled R",    "%.3f", scaledR);
        telemetry.addData("Scaled G",    "%.3f", scaledG);
        telemetry.addData("Scaled B",    "%.3f", scaledB);
        telemetry.addData("Detected",    detected.toString());
        telemetry.update();
    }

    // ------------------ Helper methods ------------------

    private double getDistanceCm() {
        DistanceSensor d = (DistanceSensor) sensor;
        return d.getDistance(DistanceUnit.CM);
    }

    private double[] normalizedRGB() {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        double sum = r + g + b;
        if (sum <= 0) return new double[]{0.0, 0.0, 0.0};
        return new double[]{ r / sum, g / sum, b / sum };
    }
}
