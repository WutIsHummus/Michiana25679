package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;

@TeleOp(name = "Limelight Data Test", group = "Test")
// @Disabled // Uncomment this line to hide from OpMode list
public class LimelightDataTest extends LinearOpMode {

    private MotorControl.Limelight limelight;
    private static final String PRIMARY_TARGET_COLOR = "blue"; // Or "red", "yellow" - match your needs

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Limelight Data Test...");
        telemetry.update();

        // Initialize the Limelight from your MotorControl class
        // Note: MotorControl itself is not fully initialized here, only its Limelight subclass.
        // If MotorControl's constructor does more hardware mapping that's not needed,
        // you might consider making Limelight initializable without a full MotorControl instance,
        // but for now, this matches your existing structure.
        try {
            limelight = new MotorControl.Limelight(hardwareMap, telemetry);
            limelight.setPrimaryClass(PRIMARY_TARGET_COLOR);
            telemetry.addLine("Limelight initialized.");
        } catch (Exception e) {
            telemetry.addLine("Error initializing Limelight: " + e.getMessage());
            telemetry.update();
            // Wait for a bit so the message can be read, then exit.
            sleep(5000);
            return;
        }

        telemetry.addData("Limelight Primary Class", PRIMARY_TARGET_COLOR);
        telemetry.addLine("Ready to start. Press Play.");
        telemetry.addLine("OpMode will continuously try to collect and display Limelight data.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Start the sample collection process (this resets buffers in your Limelight class)
            limelight.startCollectingSamples();

            while (opModeIsActive()) {
                // Attempt to collect a new set of samples (which runs the full PipeA -> PipeB logic)
                boolean newDataCollectedSuccessfully = limelight.collectSamples();

                if (newDataCollectedSuccessfully) {
                    // Get the processed data
                    // If maxSamples in your Limelight class is 1, these are effectively the latest readings.
                    // If maxSamples > 1, these are averages over the last 'maxSamples' readings.
                    Vector2d poseData = limelight.getAveragePose(); // Contains (Horizontal, Forward)
                    double angleData = limelight.getAverageAngle();

                    telemetry.addLine("--- Processed Limelight Data ---");
                    telemetry.addData("Horizontal Offset (X)", String.format("%.2f", poseData.x));
                    telemetry.addData("Forward Distance (Y)", String.format("%.2f", poseData.y));
                    telemetry.addData("Angle", String.format("%.2f", angleData));
                    telemetry.addLine("Status: Successfully processed new data.");
                } else {
                    // collectSamples() returned false, meaning a full sample wasn't processed
                    // (e.g., an error in pipeline A or B, no target found, etc.)
                    // The internal telemetry in your limelight.collectSamples() method should provide more details.
                    telemetry.addLine("--- Waiting for Limelight Data ---");
                    telemetry.addLine("Status: collectSamples() returned false. Check internal LL telemetry.");
                    // Display last known good values if you want, or clear them
                    Vector2d poseData = limelight.getAveragePose();
                    double angleData = limelight.getAverageAngle();
                    telemetry.addData("Last Avg Horiz", String.format("%.2f", poseData.x));
                    telemetry.addData("Last Avg Fwd", String.format("%.2f", poseData.y));
                    telemetry.addData("Last Avg Angle", String.format("%.2f", angleData));
                }

                // Add any other general telemetry
                telemetry.addData("Looping", "Active");
                telemetry.update();

                // Give the CPU a moment, and allow Limelight processing time
                sleep(50); // Adjust as needed, e.g., 20-100ms
            }
        }

        // Stop Limelight polling when OpMode ends
        if (limelight != null) {
            limelight.stop();
        }
    }
}