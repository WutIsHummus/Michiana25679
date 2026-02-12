package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.helpers.hardware.optimization.LoopOptimizations.BulkCacheManager;
import org.firstinspires.ftc.teamcode.helpers.hardware.optimization.LoopOptimizations.HardwareWriteCache;
import org.firstinspires.ftc.teamcode.helpers.hardware.optimization.LoopOptimizations.TelemetryThrottler;

public class ActionOpMode extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    public List<Action> runningActions = new ArrayList<>();
    private final List<Action> nextActions = new ArrayList<>();
    private BulkCacheManager bulkCache;
    private TelemetryThrottler telemetryThrottler;

    @Override
    public void init() {
        HardwareWriteCache.clear(); // Reset cached writes on init to avoid stale values
        bulkCache = new BulkCacheManager(hardwareMap);
        telemetryThrottler = new TelemetryThrottler(10.0); // ~10 Hz dashboard updates
        telemetry.addLine("Initializing...");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (bulkCache != null) {
            bulkCache.clear(); // Manual bulk cache clear once per loop
        }

        TelemetryPacket packet = new TelemetryPacket();
        updateAsync(packet);
        if (telemetryThrottler == null || telemetryThrottler.shouldUpdate()) {
            dash.sendTelemetryPacket(packet);
        }
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
        nextActions.clear();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                nextActions.add(action);
            }
        }
        runningActions = newActionsView();
    }

    protected void run(Action a) {
        runningActions.add(a);
    }

    protected void stop(Action a) {
        runningActions.remove(a);
    }


    protected boolean opModeIsActive() {
        return !isStopRequested();
    }

    private boolean isStopRequested() {
        return Thread.currentThread().isInterrupted();
    }

    private List<Action> newActionsView() {
        // Preserve existing list type but reuse backing storage for nextActions
        if (runningActions instanceof ArrayList) {
            ((ArrayList<Action>) runningActions).clear();
            ((ArrayList<Action>) runningActions).addAll(nextActions);
            return runningActions;
        }
        return new ArrayList<>(nextActions);
    }
}
