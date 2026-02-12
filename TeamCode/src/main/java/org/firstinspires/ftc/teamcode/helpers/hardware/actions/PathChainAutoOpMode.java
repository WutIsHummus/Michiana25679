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
    public static double HEADING_TOLERANCE = Math.toRadians(2); // Used by specific TurnTask implementations presumably

    protected List<BaseTask> tasks = new ArrayList<>();
    protected int currentTaskIndex = 0;
    protected int taskPhase = 0; // 0 = Primary Action (Path/Turn/ActionOnly), 1 = WAITING (for waitTime, WaitActions, WaitCondition)

    // NEW: To track actions run during the current task's wait phase
    private List<Action> currentTaskWaitPhaseActions = new ArrayList<>();

    // =========================================================
    // NEW: Background (persistent) actions that run every cycle
    // =========================================================
    protected final List<Action> backgroundActions = new ArrayList<>();
    private boolean backgroundActionsStarted = false;

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

    // =========================================================
    // NEW: Register background actions (call from OpMode init)
    // =========================================================
    protected void addBackgroundAction(Action action) {
        if (action != null) backgroundActions.add(action);
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

        // NEW: background action start gating
        backgroundActionsStarted = false;

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
        super.loop(); // Call to ActionOpMode's loop, which handles runningActions

        // =========================================================
        // NEW: Start background actions ONCE; ActionOpMode ticks them
        // =========================================================
        if (!backgroundActionsStarted) {
            for (Action a : backgroundActions) {
                run(a);
            }
            backgroundActionsStarted = true;
        }

        runTasks();
        // Add telemetry updates here if needed
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
        }
    }

    private void transitionToWaitPhaseOrAdvance(BaseTask currentTask) {
        if (canSkipWaitingPhase(currentTask)) {
            advanceToNextTask();
        } else {
            taskPhase = 1; // Move to WAITING phase
            waitTimer.resetTimer();   // Timer for overall wait duration
            actionTimer.resetTimer(); // Timer for triggering WaitActions
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
            startPath(pathTask); // Implemented by subclass
            pathTask.initiated = true;
            resetTimersAndActionsForTask(pathTask);
        } else {
            if (!isPathActive()) { // Path completed
                transitionToWaitPhaseOrAdvance(pathTask);
            }
        }
    }

    private void handleTurnTask(TurnTask turnTask) {
        if (!turnTask.initiated || isPathActive()) {
            if (isTurning() || isPathActive()) return;
            startTurn(turnTask);
            turnTask.initiated = true;
            resetTimersAndActionsForTask(turnTask);
            actionTimer.resetTimer();
        } else {
            if (actionTimer.getElapsedTimeSeconds() > turnTask.turnTimeout) {
                turnTask.timedOut = true;
                advanceToNextTask();
                return;
            }
            if (!isTurning()) { // Turn completed
                transitionToWaitPhaseOrAdvance(turnTask);
            }
        }
    }

    private void handleActionOnlyTask(ActionOnlyTask actionTask) {
        if (!actionTask.actionHasRun) {
            if (actionTask.actionToRun != null) {
                run(actionTask.actionToRun);
            }
            actionTask.actionHasRun = true;
            resetTimersAndActionsForTask(actionTask);
            transitionToWaitPhaseOrAdvance(actionTask);
        }
    }

    private void handleWaitingPhase(BaseTask currentTask) {
        double overallWaitElapsed = waitTimer.getElapsedTimeSeconds();
        double actionElapsed = actionTimer.getElapsedTimeSeconds();

        if (currentTask.timedOut) {
            advanceToNextTask();
            return;
        }

        if (overallWaitElapsed >= currentTask.maxWaitTime) {
            currentTask.timedOut = true;
            advanceToNextTask();
            return;
        }

        for (WaitAction wa : currentTask.waitActions) {
            if (!wa.triggered && wa.shouldTrigger(actionElapsed)) {
                currentTaskWaitPhaseActions.add(wa.action);
                run(wa.action);
                wa.triggered = true;
            }
        }

        boolean readyToAdvance = false;
        if (currentTask.waitCondition != null) {
            if (currentTask.conditionMetTime == null) {
                if (currentTask.waitCondition.isMet()) {
                    currentTask.conditionMetTime = overallWaitElapsed;
                }
            }
            if (currentTask.conditionMetTime != null &&
                    (overallWaitElapsed - currentTask.conditionMetTime >= currentTask.waitTime)) {
                readyToAdvance = true;
            }
        } else {
            if (overallWaitElapsed >= currentTask.waitTime) {
                readyToAdvance = true;
            }
        }

        if (readyToAdvance) {
            advanceToNextTask();
        }
    }

    // --- Task Creation Helper Methods ---
    protected PathChainTask addPath(Object pathChain, double waitTimeAfterPath) {
        PathChainTask task = new PathChainTask(pathChain, waitTimeAfterPath);
        tasks.add(task);
        return task;
    }

    public TurnTask addTurnTo(double targetAngleRadians, double waitTimeAfterTurn) {
        TurnTask task = new TurnTask(targetAngleRadians, false, waitTimeAfterTurn);
        tasks.add(task);
        return task;
    }

    public TurnTask addTurnToDegrees(double targetAngleDegrees, double waitTimeAfterTurn, double initialAngleDegrees) {
        TurnTask task = new TurnTask(targetAngleDegrees, true, waitTimeAfterTurn, initialAngleDegrees);
        tasks.add(task);
        return task;
    }

    public TurnTask addTurnToDegrees(double targetAngleDegrees, double waitTimeAfterTurn) {
        TurnTask task = new TurnTask(targetAngleDegrees, true, waitTimeAfterTurn);
        tasks.add(task);
        return task;
    }

    public TurnTask addRelativeTurn(double angleRadians, boolean isLeft, double waitTimeAfterTurn) {
        TurnTask task = new TurnTask(angleRadians, false, isLeft, waitTimeAfterTurn);
        tasks.add(task);
        return task;
    }

    public TurnTask addRelativeTurnDegrees(double angleDegrees, boolean isLeft, double waitTimeAfterTurn) {
        TurnTask task = new TurnTask(angleDegrees, true, isLeft, waitTimeAfterTurn);
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
        public double waitTime;
        public double maxWaitTime = 10.0;
        public List<WaitAction> waitActions = new ArrayList<>();
        public WaitCondition waitCondition;
        public Double conditionMetTime = null;
        public boolean timedOut = false;

        public BaseTask(double waitTime) {
            this.waitTime = waitTime;
        }

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
                wa.triggered = false;
            }
            conditionMetTime = null;
            timedOut = false;
        }
    }

    public static class PathChainTask extends BaseTask {
        public Object pathChain;
        public boolean initiated = false;

        public PathChainTask(Object pathChain, double waitTime) {
            super(waitTime);
            this.pathChain = pathChain;
        }
    }

    public static class TurnTask extends BaseTask {
        public double angle;
        public boolean useDegrees;
        public boolean isRelative;
        public boolean isLeft;
        public boolean initiated = false;
        public double initialAngleDegrees;
        public double turnTimeout = 1.0;

        public TurnTask(double angle, boolean useDegrees, double waitTime) {
            super(waitTime);
            this.angle = angle;
            this.useDegrees = useDegrees;
            this.isRelative = false;
            this.isLeft = false;
        }

        public TurnTask(double angle, boolean useDegrees, double waitTime, double initialAngleDegrees) {
            super(waitTime);
            this.angle = angle;
            this.useDegrees = useDegrees;
            this.isRelative = false;
            this.isLeft = false;
            this.initialAngleDegrees = initialAngleDegrees;
        }

        public TurnTask(double angle, boolean useDegrees, boolean isLeft, double waitTime) {
            super(waitTime);
            this.angle = angle;
            this.useDegrees = useDegrees;
            this.isRelative = true;
            this.isLeft = isLeft;
        }
    }

    public static class ActionOnlyTask extends BaseTask {
        public Action actionToRun;
        public boolean actionHasRun = false;

        public ActionOnlyTask(Action action, double waitTimeAfterAction) {
            super(waitTimeAfterAction);
            this.actionToRun = action;
        }
    }

    public static class WaitAction {
        Double triggerTime;
        WaitCondition condition;
        public Action action;
        public boolean triggered;

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

        public boolean shouldTrigger(double elapsedActionTimerSeconds) {
            boolean timeCriteriaMet = (triggerTime != null && elapsedActionTimerSeconds >= triggerTime);
            boolean conditionCriteriaMet = (condition != null && condition.isMet());

            if (triggerTime != null && condition != null) {
                return timeCriteriaMet || conditionCriteriaMet;
            } else if (triggerTime != null) {
                return timeCriteriaMet;
            } else if (condition != null) {
                return conditionCriteriaMet;
            }
            return false;
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
