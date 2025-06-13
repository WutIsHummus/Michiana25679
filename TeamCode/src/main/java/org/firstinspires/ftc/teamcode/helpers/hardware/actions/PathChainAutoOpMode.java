package org.firstinspires.ftc.teamcode.helpers.hardware.actions; // Ensure this matches your package structure

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.ArrayList;
import java.util.List;

public abstract class PathChainAutoOpMode extends ActionOpMode { // Make sure ActionOpMode is accessible

    protected final FtcDashboard dashboard = FtcDashboard.getInstance();

    protected Timer waitTimer = new Timer();
    protected Timer actionTimer = new Timer();
    protected Timer pathTimer = new Timer(); // Declared, but core logic uses waitTimer/actionTimer for task waits

    public double targetHeadingRadians; // Used by specific TurnTask implementations presumably
    protected boolean isActivelyTurningInternalFlag = false; // Used by specific TurnTask implementations presumably
    public static double HEADING_TOLERANCE = Math.toRadians(1.0); // Used by specific TurnTask implementations presumably

    protected List<BaseTask> tasks = new ArrayList<>();
    protected int currentTaskIndex = 0;
    protected int taskPhase = 0; // 0 = Primary Action (Path/Turn/ActionOnly), 1 = WAITING (for waitTime, WaitActions, WaitCondition)

    // NEW: To track actions run during the current task's wait phase
    private List<Action> currentTaskWaitPhaseActions = new ArrayList<>();

    // Abstract methods to be implemented by subclasses
    protected abstract void buildPathChains(); // Likely for older structure, might be replaced by buildTaskList

    protected abstract void buildTaskList();   // Primary method to define the sequence of tasks

    protected abstract boolean isPathActive();  // Checks if a Roadrunner trajectory (from PathChainTask) is running

    protected abstract boolean isTurning();     // Checks if a turn operation (from TurnTask) is active

    protected abstract double getCurrentTValue(); // Potentially for telemetry or advanced path following logic

    protected abstract void startPath(PathChainTask task); // Initiates the path following for a PathChainTask

    protected abstract void startTurn(TurnTask task);      // Initiates the turn for a TurnTask

    public void setHeadingTolerance(double tolerance) {
        HEADING_TOLERANCE = tolerance;
    }

    @Override
    public void init() {
        super.init(); // Call to ActionOpMode's init
        // buildTaskList should be called here by the concrete OpMode
        // e.g., in MyAutoOpMode.init():
        //   super.init();
        //   buildTaskList();
    }

    @Override
    public void start() {
        super.start(); // Call to ActionOpMode's start
        if (waitTimer == null) waitTimer = new Timer();
        else waitTimer.resetTimer();
        if (actionTimer == null) actionTimer = new Timer();
        else actionTimer.resetTimer();
        if (pathTimer == null) pathTimer = new Timer();
        else pathTimer.resetTimer();

        currentTaskIndex = 0;
        taskPhase = 0;
        isActivelyTurningInternalFlag = false; // Reset any state flags

        for (BaseTask task : tasks) {
            // Reset initiation/completion flags for each task type
            if (task instanceof TurnTask) {
                ((TurnTask) task).initiated = false;
            } else if (task instanceof PathChainTask) {
                ((PathChainTask) task).initiated = false;
            } else if (task instanceof ActionOnlyTask) {
                ((ActionOnlyTask) task).actionHasRun = false;
            }
            task.resetWaitActions(); // MODIFIED: This now also resets the task.timedOut flag
        }
    }

    @Override
    public void loop() {
        super.loop(); // Call to ActionOpMode's loop, which likely handles runningActions
        runTasks();
        // Add telemetry updates here if needed
        // telemetry.addData("Current Task Index", currentTaskIndex);
        // telemetry.addData("Task Phase", taskPhase == 0 ? "ACTION" : "WAITING");
        // if (currentTaskIndex < tasks.size()) {
        //     telemetry.addData("Task Type", tasks.get(currentTaskIndex).getClass().getSimpleName());
        //     telemetry.addData("Task WaitTime", tasks.get(currentTaskIndex).waitTime);
        //     telemetry.addData("Task MaxWaitTime", tasks.get(currentTaskIndex).maxWaitTime);
        //     telemetry.addData("Task TimedOut", tasks.get(currentTaskIndex).timedOut);
        //     telemetry.addData("WaitTimer (Overall)", waitTimer.getElapsedTimeSeconds());
        //     telemetry.addData("ActionTimer (For Triggers)", actionTimer.getElapsedTimeSeconds());
        // }
        // telemetry.update();
    }

    protected void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            // No more tasks, autonomous sequence is complete
            // requestOpModeStop(); // Or handle completion as needed
            return;
        }
        BaseTask currentTask = tasks.get(currentTaskIndex);
        switch (taskPhase) {
            case 0: // Primary Action Phase (Path, Turn, or singular Action)
                if (currentTask instanceof PathChainTask) {
                    handlePathTask((PathChainTask) currentTask);
                } else if (currentTask instanceof TurnTask) {
                    handleTurnTask((TurnTask) currentTask);
                } else if (currentTask instanceof ActionOnlyTask) {
                    handleActionOnlyTask((ActionOnlyTask) currentTask);
                }
                break;
            case 1: // Waiting Phase (after primary action, for waitTime, WaitActions, WaitConditions)
                handleWaitingPhase(currentTask);
                break;
        }
    }



    private boolean canSkipWaitingPhase(BaseTask task) {
        // A task can skip its waiting phase if it has no explicit waitTime,
        // no WaitActions to perform, and no WaitCondition to meet.
        return task.waitTime == 0.0 && task.waitActions.isEmpty() && task.waitCondition == null;
    }

    private void advanceToNextTask() {
        currentTaskIndex++;
        taskPhase = 0; // Reset to ACTION phase for the new task
        if (currentTaskIndex < tasks.size()) {
            BaseTask nextTask = tasks.get(currentTaskIndex);
            // Reset initiated/completion flags for the new task
            if (nextTask instanceof TurnTask) ((TurnTask) nextTask).initiated = false;
            else if (nextTask instanceof PathChainTask)
                ((PathChainTask) nextTask).initiated = false;
            else if (nextTask instanceof ActionOnlyTask)
                ((ActionOnlyTask) nextTask).actionHasRun = false;
            nextTask.resetWaitActions(); // MODIFIED: Ensure the new task's state (including timedOut flag) is fresh
        } else {
            // All tasks completed
            // Consider adding a log or telemetry message
        }
    }

    private void transitionToWaitPhaseOrAdvance(BaseTask currentTask) {
        if (canSkipWaitingPhase(currentTask)) {
            advanceToNextTask(); // This will call cleanupCurrentTaskWaitActions
        } else {
            taskPhase = 1; // Move to WAITING phase
            waitTimer.resetTimer();  // Timer for overall wait duration (waitTime, maxWaitTime)
            actionTimer.resetTimer(); // Timer for triggering WaitActions based on their triggerTime
            // currentTask.resetWaitActions() is NOT called here; its state persists for its own waiting phase.
            // cleanupCurrentTaskWaitActions() is also not called here, as we are *entering* the wait phase.
            // It's called when the task *starts* (via resetTimersAndActionsForTask) or *ends* (via advanceToNextTask).
        }
    }

    private void resetTimersAndActionsForTask(BaseTask task) {
        waitTimer.resetTimer();
        actionTimer.resetTimer();
        // pathTimer.resetTimer(); // If pathTimer has specific per-task use, reset it too.
    }

    private void handlePathTask(PathChainTask pathTask) {
        if (!pathTask.initiated) {
            if (isPathActive()) return; // Wait for previous path/turn to complete if any confusion
            startPath(pathTask); // Implemented by subclass, starts the Roadrunner trajectory
            pathTask.initiated = true;
            resetTimersAndActionsForTask(pathTask); // Resets timers and prepares for this task's own wait phase
        } else {
            // Path has been initiated, check if it's still active
            if (!isPathActive()) { // Path completed
                transitionToWaitPhaseOrAdvance(pathTask); // Move to waiting phase or next task
            }
            // If path is still active, do nothing, let it run.
        }
    }

    private void handleTurnTask(TurnTask turnTask) {
        if (!turnTask.initiated) {
            if (isTurning() || isPathActive()) return; // Wait for previous turn/path
            startTurn(turnTask); // Implemented by subclass, starts the turn
            resetTimersAndActionsForTask(turnTask); // Resets timers and prepares for this task's own wait phase
        } else {
            // Turn has been initiated, check if it's still active
            if (!isTurning()) { // Turn completed
                transitionToWaitPhaseOrAdvance(turnTask); // Move to waiting phase or next task
            }
            // If turn is still active, do nothing, let it run.
        }
    }

    private void handleActionOnlyTask(ActionOnlyTask actionTask) {
        if (!actionTask.actionHasRun) {
            // Assuming ActionOnlyTask's primary action is relatively quick
            // or is managed by ActionOpMode's main action loop if it's a Roadrunner Action
            if (actionTask.actionToRun != null) {
                run(actionTask.actionToRun); // run() is from ActionOpMode, adds to its internal list
            }
            actionTask.actionHasRun = true; // Mark primary action as dispatched/run
            resetTimersAndActionsForTask(actionTask); // Resets timers and prepares for this task's own wait phase
            transitionToWaitPhaseOrAdvance(actionTask); // Immediately move to its waiting phase or next task
        }
        // If actionHasRun is true, it means we are either in its waiting phase (if any)
        // or it has already advanced. The taskPhase will be 1 if it entered a waiting phase.
    }

    private void handleWaitingPhase(BaseTask currentTask) {
        double overallWaitElapsed = waitTimer.getElapsedTimeSeconds(); // Time since waiting phase started
        double actionElapsed = actionTimer.getElapsedTimeSeconds();   // Time for WaitAction triggers

        // MODIFIED: Centralized timeout and advancement logic

        // 1. Check if task already marked as timedOut (e.g. from a previous cycle hitting maxWaitTime)
        if (currentTask.timedOut) {
            advanceToNextTask(); // This will call cleanupCurrentTaskWaitActions
            return;
        }

        // 2. Check for maxWaitTime timeout for the current task's waiting phase
        if (overallWaitElapsed >= currentTask.maxWaitTime) {
            currentTask.timedOut = true; // Mark the task as timed out
            advanceToNextTask(); // This will call cleanupCurrentTaskWaitActions
            return; // Exit immediately
        }

        // 3. Process WaitActions if not timed out
        // These actions are run in parallel with the main wait timer (overallWaitElapsed)
        for (WaitAction wa : currentTask.waitActions) {
            if (!wa.triggered && wa.shouldTrigger(actionElapsed)) {
                // Task is not timed out at this point in the current cycle, safe to start action
                currentTaskWaitPhaseActions.add(wa.action); // Track this action
                run(wa.action); // Run it using ActionOpMode's run method
                wa.triggered = true;
            }
        }

        // 4. Check for normal advancement conditions (waitTime or waitCondition met)
        boolean readyToAdvance = false;
        if (currentTask.waitCondition != null) { // Task has a specific condition to meet
            if (currentTask.conditionMetTime == null) { // Condition not yet met
                if (currentTask.waitCondition.isMet()) {
                    currentTask.conditionMetTime = overallWaitElapsed; // Record time condition was met
                }
                // If condition not met, and not timed out by maxWaitTime, we just continue waiting.
            }
            // If condition was met, check if we've waited the post-condition 'waitTime'
            if (currentTask.conditionMetTime != null &&
                    (overallWaitElapsed - currentTask.conditionMetTime >= currentTask.waitTime)) {
                readyToAdvance = true;
            }
        } else { // No specific wait condition, just a simple timed wait for this phase
            if (overallWaitElapsed >= currentTask.waitTime) {
                readyToAdvance = true;
            }
        }

        if (readyToAdvance) {
            advanceToNextTask(); // This will call cleanupCurrentTaskWaitActions, interrupting ongoing WaitActions
        }
        // If not readyToAdvance and not timedOut, we simply continue in the waiting phase.
        // OpMode loop will call this method again.
    }

    // --- Task Creation Helper Methods ---
    protected PathChainTask addPath(Object pathChain, double waitTimeAfterPath) {
        PathChainTask task = new PathChainTask(pathChain, waitTimeAfterPath);
        tasks.add(task);
        return task;
    }

    public TurnTask addTurnTo(double targetAngleRadians, double waitTimeAfterTurn) {
        TurnTask task = new TurnTask(targetAngleRadians, false, waitTimeAfterTurn); // false for radians
        tasks.add(task);
        return task;
    }

    public TurnTask addTurnToDegrees(double targetAngleDegrees, double waitTimeAfterTurn, double initialAngleDegrees) {
        TurnTask task = new TurnTask(targetAngleDegrees, true, waitTimeAfterTurn,initialAngleDegrees); // true for degrees
        tasks.add(task);
        return task;
    }

    public TurnTask addTurnToDegrees(double targetAngleDegrees, double waitTimeAfterTurn) {
        TurnTask task = new TurnTask(targetAngleDegrees, true, waitTimeAfterTurn); // true for degrees
        tasks.add(task);
        return task;
    }

    public TurnTask addRelativeTurn(double angleRadians, boolean isLeft, double waitTimeAfterTurn) {
        TurnTask task = new TurnTask(angleRadians, false, isLeft, waitTimeAfterTurn); // false for radians
        tasks.add(task);
        return task;
    }

    public TurnTask addRelativeTurnDegrees(double angleDegrees, boolean isLeft, double waitTimeAfterTurn) {
        TurnTask task = new TurnTask(angleDegrees, true, isLeft, waitTimeAfterTurn); // true for degrees
        tasks.add(task);
        return task;
    }

    protected ActionOnlyTask addAction(Action action, double waitTimeAfterAction) {
        ActionOnlyTask task = new ActionOnlyTask(action, waitTimeAfterAction);
        tasks.add(task);
        return task;
    }


    // --- Inner Classes for Task Definitions ---

    public abstract static class BaseTask {
        public double waitTime; // Time to wait in the WAITING phase, or after a condition is met
        public double maxWaitTime = 10.0; // Absolute max time for the WAITING phase (default 10s)
        public List<WaitAction> waitActions = new ArrayList<>(); // Actions to run during WAITING phase
        public WaitCondition waitCondition; // Optional condition to be met during WAITING phase
        public Double conditionMetTime = null; // Timestamp when waitCondition was met
        public boolean timedOut = false; // NEW FIELD: True if this task's wait phase hit maxWaitTime

        public BaseTask(double waitTime) {
            this.waitTime = waitTime;
        }

        // Fluent interface for adding WaitActions and conditions
        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T addWaitAction(double triggerTimeSeconds, Action action) {
            waitActions.add(new WaitAction(triggerTimeSeconds, action));
            return (T) this;
        }

        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T addWaitAction(WaitCondition condition, Action action) {
            waitActions.add(new WaitAction(condition, action));
            return (T) this;
        }

        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T addWaitAction(double triggerTimeSeconds, WaitCondition condition, Action action) {
            waitActions.add(new WaitAction(triggerTimeSeconds, condition, action));
            return (T) this;
        }

        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T setWaitCondition(WaitCondition condition) {
            this.waitCondition = condition;
            return (T) this;
        }

        @SuppressWarnings("unchecked")
        public <T extends BaseTask> T setMaxWaitTime(double maxWaitTimeSeconds) {
            this.maxWaitTime = maxWaitTimeSeconds;
            return (T) this;
        }

        public void resetWaitActions() {
            for (WaitAction wa : waitActions) {
                wa.triggered = false; // Reset triggered status for all wait actions
            }
            conditionMetTime = null; // Reset condition met time
            timedOut = false; // MODIFIED: Reset the timedOut flag
        }
    }

    public static class PathChainTask extends BaseTask {
        public Object pathChain; // Could be a Roadrunner Trajectory, TrajectorySequence, or custom path object
        public boolean initiated = false; // True once startPath() has been called for this task

        public PathChainTask(Object pathChain, double waitTime) {
            super(waitTime);
            this.pathChain = pathChain;
        }
    }

    public static class TurnTask extends BaseTask {
        public double angle;
        public boolean useDegrees;
        public boolean isRelative; // Is the turn relative to current heading or absolute field heading?
        public boolean isLeft;     // If relative, specifies direction (meaningful for some turn implementations)
        public boolean initiated = false; // True once startTurn() has been called
        public double initialAngleDegrees;

        // Constructor for absolute turns
        public TurnTask(double angle, boolean useDegrees, double waitTime) {
            super(waitTime);
            this.angle = angle;
            this.useDegrees = useDegrees;
            this.isRelative = false;
            this.isLeft = false; // Not applicable for absolute turns typically
        }

        public TurnTask(double angle, boolean useDegrees, double waitTime, double initialAngleDegrees) {
            super(waitTime);
            this.angle = angle;
            this.useDegrees = useDegrees;
            this.isRelative = false;
            this.isLeft = false; // Not applicable for absolute turns typically
            this.initialAngleDegrees = initialAngleDegrees;
        }

        // Constructor for relative turns
        public TurnTask(double angle, boolean useDegrees, boolean isLeft, double waitTime) {
            super(waitTime);
            this.angle = angle;
            this.useDegrees = useDegrees;
            this.isRelative = true;
            this.isLeft = isLeft;
        }
    }

    public static class ActionOnlyTask extends BaseTask {
        public Action actionToRun; // The primary Roadrunner Action for this task
        public boolean actionHasRun = false; // True once actionToRun has been dispatched

        public ActionOnlyTask(Action action, double waitTimeAfterAction) {
            super(waitTimeAfterAction); // waitTime here is for the phase *after* actionToRun is dispatched
            this.actionToRun = action;
        }
    }

    public static class WaitAction {
        Double triggerTime;      // Time in seconds (on actionTimer) to trigger this action
        WaitCondition condition; // Optional condition to trigger this action
        public Action action;    // The Roadrunner Action to execute
        public boolean triggered; // True once this WaitAction has been triggered

        public WaitAction(double triggerTime, Action action) {
            this(triggerTime, null, action);
        }

        public WaitAction(WaitCondition condition, Action action) {
            this(null, condition, action);
        }

        public WaitAction(Double triggerTime, WaitCondition condition, Action action) {
            this.triggerTime = triggerTime;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }

        // Determines if this WaitAction should be triggered based on elapsed time or condition
        public boolean shouldTrigger(double elapsedActionTimerSeconds) {
            boolean timeCriteriaMet = (triggerTime != null && elapsedActionTimerSeconds >= triggerTime);
            boolean conditionCriteriaMet = (condition != null && condition.isMet());

            if (triggerTime != null && condition != null) { // Both time and condition specified
                return timeCriteriaMet || conditionCriteriaMet; // Trigger if EITHER is met
            } else if (triggerTime != null) { // Only time specified
                return timeCriteriaMet;
            } else if (condition != null) { // Only condition specified
                return conditionCriteriaMet;
            }
            return false; // No trigger criteria defined (should not happen with current constructors)
        }
    }

    @FunctionalInterface
    public interface WaitCondition {
        boolean isMet();
    }

    public static class Timer {
        private long startTimeNanos;

        public Timer() {
            resetTimer();
        }

        public void resetTimer() {
            startTimeNanos = System.nanoTime();
        }

        public double getElapsedTimeSeconds() {
            return (System.nanoTime() - startTimeNanos) / 1_000_000_000.0;
        }

        public long getElapsedTimeMillis() {
            return (System.nanoTime() - startTimeNanos) / 1_000_000;
        }
    }
}