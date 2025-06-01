package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;

import java.util.ArrayList;
import java.util.List;

/**
 * An abstract autonomous OpMode that supports a series of PathChainTasks.
 *
 * Subclasses should override buildPathChains() and buildTaskList() to define:
 *  - The geometry of your paths (via buildPathChains)
 *  - The sequence of tasks (via buildTaskList)
 *
 * This version does not include any waiting logic.
 */
public abstract class Fastpathchain extends ActionOpMode {

    // Dashboard instance (if desired)
    protected final FtcDashboard dashboard = FtcDashboard.getInstance();

    // ---------- Task Management Fields ----------
    protected Timer pathTimer = new Timer();

    protected List<PathChainTask> tasks = new ArrayList<>();
    protected int currentTaskIndex = 0;
    // Parameter value when you consider the path complete (adjust as needed)
    protected double PATH_COMPLETION_T = 0.985;

    // ---------- Abstract Methods ----------
    /**
     * Override this method to build your path geometries.
     */
    protected abstract void buildPathChains();

    /**
     * Override this method to build your sequence of PathChainTasks.
     */
    protected abstract void buildTaskList();

    // ---------- OpMode Lifecycle Methods ----------
    @Override
    public void init() {
        super.init();
        // Subclasses are expected to call buildPathChains() and buildTaskList() later in their init.
    }

    @Override
    public void loop() {
        super.loop();
        runTasks();
    }

    // ---------- Task Runner Logic ----------
    /**
     * runTasks() is called in the loop to drive the current PathChainTask.
     */
    protected void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            return; // All tasks completed.
        }

        PathChainTask currentTask = tasks.get(currentTaskIndex);

        if (!isPathActive()) {
            startPath(currentTask);
        }
        double tValue = getCurrentTValue();
        if (tValue >= PATH_COMPLETION_T) {
            currentTaskIndex++;
        }
    }

    /**
     * Dummy method to indicate whether the path is active.
     * Subclasses should override this to integrate with your path follower.
     */
    protected boolean isPathActive() {
        return false;
    }

    /**
     * Dummy method to return the current progress along the path.
     * Subclasses should override this with actual progress logic.
     */
    protected double getCurrentTValue() {
        return 0.0;
    }

    /**
     * Dummy method to start a path.
     * Subclasses should override this to command the follower to begin following the path.
     */
    protected void startPath(PathChainTask task) {
    }

    // ---------- Helper Inner Classes ----------

    /**
     * PathChainTask represents one task in your autonomous routine.
     *
     * The waitTime and wait-related methods have been retained only for structure.
     * They are no longer used by the autonomous routine.
     */
    public static class PathChainTask {
        // Replace Object with your actual PathChain type if desired.
        public Object pathChain;
        public double waitTime; // Not used.
        public List<WaitAction> waitActions = new ArrayList<>();
        public WaitCondition waitCondition; // Not used.
        public Double conditionMetTime = null; // Not used.

        public PathChainTask(Object pathChain, double waitTime) {
            this.pathChain = pathChain;
            this.waitTime = waitTime;
        }

        /**
         * Add a wait action. This method remains for compatibility but will not be used.
         */
        public PathChainTask addWaitAction(double triggerTime, Action action) {
            waitActions.add(new WaitAction(triggerTime, action));
            return this;
        }

        /**
         * Add a wait action. This method remains for compatibility but will not be used.
         */
        public PathChainTask addWaitAction(WaitCondition condition, Action action) {
            waitActions.add(new WaitAction(condition, action));
            return this;
        }

        /**
         * Add a wait action. This method remains for compatibility but will not be used.
         */
        public PathChainTask addWaitAction(double triggerTime, WaitCondition condition, Action action) {
            waitActions.add(new WaitAction(triggerTime, condition, action));
            return this;
        }

        /**
         * Set an overall wait condition for this task.
         * Remains for compatibility but is not used.
         */
        public PathChainTask setWaitCondition(WaitCondition condition) {
            this.waitCondition = condition;
            return this;
        }

        /**
         * Set a maximum wait time (timeout) for this task when using an overall condition.
         * Remains for compatibility but is not used.
         */
        public PathChainTask setMaxWaitTime(double maxWaitTime) {
            return this;
        }

        /**
         * Resets all wait actions and clears the condition-met timer.
         * Remains for compatibility but is not used.
         */
        public void resetWaitActions() {
            for (WaitAction wa : waitActions) {
                wa.triggered = false;
            }
            conditionMetTime = null;
        }
    }

    /**
     * WaitAction and WaitCondition remain defined for compatibility.
     */
    public static class WaitAction {
        Double triggerTime; // in seconds; may be null
        WaitCondition condition; // may be null
        public Action action;
        public boolean triggered;

        public WaitAction(double triggerTime, Action action) {
            this.triggerTime = triggerTime;
            this.action = action;
            this.condition = null;
            this.triggered = false;
        }

        public WaitAction(WaitCondition condition, Action action) {
            this.triggerTime = null;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }

        public WaitAction(double triggerTime, WaitCondition condition, Action action) {
            this.triggerTime = triggerTime;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }
    }

    @FunctionalInterface
    public interface WaitCondition {
        boolean isMet();
    }

    /**
     * A simple Timer helper class.
     */
    public static class Timer {
        private long startTime;

        public Timer() {
            resetTimer();
        }

        public void resetTimer() {
            startTime = System.currentTimeMillis();
        }

        /**
         * Returns the elapsed time in seconds.
         */
        public double getElapsedTimeSeconds() {
            return (System.currentTimeMillis() - startTime) / 1000.0;
        }
    }
}
