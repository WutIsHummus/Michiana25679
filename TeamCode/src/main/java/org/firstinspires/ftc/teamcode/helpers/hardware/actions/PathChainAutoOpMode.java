package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * An abstract autonomous OpMode that supports a series of PathChainTasks and TurnTasks.
 *
 * Subclasses should override buildPathChains(), buildTaskList(), and the follower methods to define:
 *  - The geometry of your paths (via buildPathChains)
 *  - The sequence of tasks with any wait actions/conditions (via buildTaskList)
 *  - Integration with your Follower instance
 *
 * This version supports both path following and turning operations.
 */

public abstract class PathChainAutoOpMode extends ActionOpMode {

    // Dashboard instance (if desired)
    protected final FtcDashboard dashboard = FtcDashboard.getInstance();

    // ---------- Task Management Fields ----------
    protected Timer pathTimer = new Timer();
    protected Timer waitTimer = new Timer();    // used for overall waiting
    protected Timer actionTimer = new Timer();  // used for triggering wait actions

    protected List<BaseTask> tasks = new ArrayList<>();
    protected int currentTaskIndex = 0;
    // taskPhase: 0 = DRIVING/TURNING phase, 1 = WAITING phase.
    protected int taskPhase = 0;
    // Parameter value when you consider the path complete (adjust as needed)
    protected double PATH_COMPLETION_T = 0.985;

    // ---------- Abstract Methods ----------
    /**
     * Override this method to build your path geometries.
     */
    protected abstract void buildPathChains();

    /**
     * Override this method to build your sequence of tasks (PathChainTasks and TurnTasks).
     */
    protected abstract void buildTaskList();

    /**
     * Override this method to integrate with your Follower instance.
     * Should return true if the follower is currently following a path.
     */
    protected abstract boolean isPathActive();

    /**
     * Override this method to integrate with your Follower instance.
     * Should return true if the follower is currently executing a turn.
     */
    protected abstract boolean isTurning();

    /**
     * Override this method to return the current progress along the path.
     */
    protected abstract double getCurrentTValue();

    /**
     * Override this method to start following a path.
     */
    protected abstract void startPath(PathChainTask task);

    /**
     * Override this method to start a turn operation.
     */
    protected abstract void startTurn(TurnTask task);

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
     * runTasks() is called in the loop to drive the current task through its DRIVING/TURNING and WAITING phases.
     */
    protected void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            return; // All tasks completed.
        }

        BaseTask currentTask = tasks.get(currentTaskIndex);

        switch (taskPhase) {
            case 0: // DRIVING/TURNING phase.
                if (currentTask instanceof PathChainTask) {
                    handlePathTask((PathChainTask) currentTask);
                } else if (currentTask instanceof TurnTask) {
                    handleTurnTask((TurnTask) currentTask);
                }
                break;

            case 1: // WAITING phase.
                handleWaitingPhase(currentTask);
                break;
        }
    }

    private void handlePathTask(PathChainTask pathTask) {
        if (!isPathActive()) {
            startPath(pathTask);
            // Reset timers for the waiting phase
            waitTimer.resetTimer();
            actionTimer.resetTimer();
            pathTask.resetWaitActions();
        }
        double tValue = getCurrentTValue();
        if (tValue >= PATH_COMPLETION_T) {
            // Transition to WAITING phase:
            waitTimer.resetTimer();
            actionTimer.resetTimer();
            taskPhase = 1;
        }
    }

    private void handleTurnTask(TurnTask turnTask) {
        if (!isTurning()) {
            startTurn(turnTask);
            // Reset timers for the waiting phase
            waitTimer.resetTimer();
            actionTimer.resetTimer();
            turnTask.resetWaitActions();
        }
        if (!isTurning()) {
            // Turn is complete, transition to WAITING phase:
            waitTimer.resetTimer();
            actionTimer.resetTimer();
            taskPhase = 1;
        }
    }

    private void handleWaitingPhase(BaseTask currentTask) {
        double overallWaitElapsed = waitTimer.getElapsedTimeSeconds();
        double actionElapsed = actionTimer.getElapsedTimeSeconds();

        // Process individual wait actions using the actionTimer.
        for (WaitAction wa : currentTask.waitActions) {
            if (!wa.triggered && wa.shouldTrigger(actionElapsed)) {
                run(wa.action);
                wa.triggered = true;
            }
        }

        // Process overall wait condition and timeouts using the waitTimer.
        if (currentTask.waitCondition != null) {
            if (currentTask.conditionMetTime == null) {
                if (currentTask.waitCondition.isMet()) {
                    currentTask.conditionMetTime = overallWaitElapsed;
                } else if (overallWaitElapsed >= currentTask.maxWaitTime) {
                    currentTaskIndex++;
                    taskPhase = 0;
                }
            } else {
                if (overallWaitElapsed - currentTask.conditionMetTime >= currentTask.waitTime) {
                    currentTaskIndex++;
                    taskPhase = 0;
                }
            }
        } else {
            if (overallWaitElapsed >= currentTask.waitTime) {
                currentTaskIndex++;
                taskPhase = 0;
            }
        }
    }

    // ---------- Helper Methods for Task Creation ----------

    /**
     * Create a path chain task.
     */
    protected PathChainTask createPathTask(Object pathChain, double waitTime) {
        return new PathChainTask(pathChain, waitTime);
    }

    /**
     * Create a turn task using radians.
     */
    protected TurnTask createTurnTask(double radians, double waitTime) {
        return new TurnTask(radians, false, waitTime);
    }

    /**
     * Create a turn task using degrees.
     */
    protected TurnTask createTurnTaskDegrees(double degrees, double waitTime) {
        return new TurnTask(degrees, true, waitTime);
    }

    /**
     * Create a relative turn task using radians.
     */
    protected TurnTask createRelativeTurnTask(double radians, boolean isLeft, double waitTime) {
        return new TurnTask(radians, false, isLeft, waitTime);
    }

    /**
     * Create a relative turn task using degrees.
     */
    protected TurnTask createRelativeTurnTaskDegrees(double degrees, boolean isLeft, double waitTime) {
        return new TurnTask(degrees, true, isLeft, waitTime);
    }

    // ---------- Helper Inner Classes ----------

    /**
     * Base class for all task types.
     */
    public abstract static class BaseTask {
        public double waitTime; // Wait time after task completion
        public double maxWaitTime = Double.MAX_VALUE; // Maximum overall wait time (timeout)
        public List<WaitAction> waitActions = new ArrayList<>();
        public WaitCondition waitCondition; // Optional overall condition.
        public Double conditionMetTime = null; // Recorded time when the overall condition was met.

        public BaseTask(double waitTime) {
            this.waitTime = waitTime;
        }

        /**
         * Add a wait action that triggers after a given time.
         */
        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T addWaitAction(double triggerTime, Action action) {
            waitActions.add(new WaitAction(triggerTime, action));
            return (T) this;
        }

        /**
         * Add a wait action that triggers when a given condition is met.
         */
        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T addWaitAction(WaitCondition condition, Action action) {
            waitActions.add(new WaitAction(condition, action));
            return (T) this;
        }

        /**
         * Add a wait action that triggers when either the specified time has elapsed or the condition is met.
         */
        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T addWaitAction(double triggerTime, WaitCondition condition, Action action) {
            waitActions.add(new WaitAction(triggerTime, condition, action));
            return (T) this;
        }

        /**
         * Set an overall wait condition for this task.
         */
        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T setWaitCondition(WaitCondition condition) {
            this.waitCondition = condition;
            return (T) this;
        }

        /**
         * Set a maximum wait time (timeout) for this task when using an overall condition.
         */
        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T setMaxWaitTime(double maxWaitTime) {
            this.maxWaitTime = maxWaitTime;
            return (T) this;
        }

        /**
         * Resets all wait actions and clears the condition-met timer.
         */
        public void resetWaitActions() {
            for (WaitAction wa : waitActions) {
                wa.triggered = false;
            }
            conditionMetTime = null;
        }
    }

    /**
     * PathChainTask represents a path following task in your autonomous routine.
     */
    public static class PathChainTask extends BaseTask {
        // Replace Object with your actual PathChain type if desired.
        public Object pathChain;

        public PathChainTask(Object pathChain, double waitTime) {
            super(waitTime);
            this.pathChain = pathChain;
        }
    }

    /**
     * TurnTask represents a turning task in your autonomous routine.
     */
    public static class TurnTask extends BaseTask {
        public double angle; // The angle to turn to or by
        public boolean useDegrees; // Whether the angle is in degrees (true) or radians (false)
        public boolean isRelative; // Whether this is a relative turn
        public boolean isLeft; // For relative turns, which direction to turn

        // Constructor for absolute turns (turnTo/turnToDegrees)
        public TurnTask(double angle, boolean useDegrees, double waitTime) {
            super(waitTime);
            this.angle = angle;
            this.useDegrees = useDegrees;
            this.isRelative = false;
            this.isLeft = false;
        }

        // Constructor for relative turns (turn/turnDegrees)
        public TurnTask(double angle, boolean useDegrees, boolean isLeft, double waitTime) {
            super(waitTime);
            this.angle = angle;
            this.useDegrees = useDegrees;
            this.isRelative = true;
            this.isLeft = isLeft;
        }
    }

    /**
     * WaitAction allows you to trigger an Action during the WAIT phase either after a given time,
     * when a given condition is met, or when either occurs.
     */
    public static class WaitAction {
        // Use boxed Double so it can be null if not set.
        Double triggerTime; // in seconds; may be null
        WaitCondition condition; // may be null
        public Action action;
        public boolean triggered;

        // Constructor: time-only trigger.
        public WaitAction(double triggerTime, Action action) {
            this.triggerTime = triggerTime;
            this.action = action;
            this.condition = null;
            this.triggered = false;
        }

        // Constructor: condition-only trigger.
        public WaitAction(WaitCondition condition, Action action) {
            this.triggerTime = null;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }

        // Constructor: both a time and condition trigger.
        public WaitAction(double triggerTime, WaitCondition condition, Action action) {
            this.triggerTime = triggerTime;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }

        /**
         * Determines whether this wait action should trigger based on the elapsed time.
         */
        public boolean shouldTrigger(double elapsedSeconds) {
            boolean timeMet = (triggerTime != null && elapsedSeconds >= triggerTime);
            boolean conditionMet = (condition != null && condition.isMet());
            return timeMet || conditionMet;
        }
    }

    /**
     * Functional interface for a wait condition.
     */
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