package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

public class ActionOpMode extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    public List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        updateAsync(packet);
        dash.sendTelemetryPacket(packet);
    }

    protected void runBlocking(Action a) {
        Canvas c = new Canvas();
        a.preview(c);

        boolean b = true;
        while (b && opModeIsActive()) {
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());

            b = a.run(p);

            dash.sendTelemetryPacket(p);
        }
    }

    protected void updateAsync(TelemetryPacket packet) {
        // Update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
    }

    protected void run(Action a) {
        runningActions.add(a);
    }

    protected void stop(Action a) {
        runningActions.remove(a);
    }


    private boolean opModeIsActive() {
        return !isStopRequested();
    }

    private boolean isStopRequested() {
        return Thread.currentThread().isInterrupted();
    }
}
