package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;

/**
 * MotorActions – updated to support ONLY the physical servos that still exist.
 * A dedicated inner class now wraps each servo (or servo‑pair) so every
 * logical mechanism exposes clearly‑named Action helpers that set the target
 * position in one line of op‑mode code.
 */
public class MotorActions {
    // === core HW interface ===
    public final MotorControl mc;
    // === subsystem helpers ===
    public final Lift lift;
    public final Extendo extendo;
    public final Spin spin;
    public final Hang hang;

    // === servo wrappers ===
    public final OuttakeLinkage linkage;
    public final OuttakeClaw    claw;
    public final OuttakeArm     outArm;
    public final IntakeArm      inArm;
    public final IntakePivot    inPivot;
    public final PtolLatch      ptolLatch;
    public final PtorLatch      ptorLatch;
    public final Sweeper        sweeper;

    public MotorActions(MotorControl mc) {
        this.mc = mc;
        lift   = new Lift();
        extendo = new Extendo();
        spin    = new Spin(mc);
        hang    = new Hang();
        sweeper = new Sweeper();

        linkage   = new OuttakeLinkage();
        claw      = new OuttakeClaw();
        outArm    = new OuttakeArm();
        inArm     = new IntakeArm();
        inPivot   = new IntakePivot();
        ptolLatch = new PtolLatch();
        ptorLatch = new PtorLatch();
    }


    public Action update() {
        return t -> {
            mc.update();
            return true; // this returns true to make it loop forever; use RaceParallelCommand
        };
    }

    public Action outtakeTransfer(){
        return new SequentialAction(
                claw.open(),
                new SleepAction(0.3),
                claw.transfer(),
                outArm.transfer(),
                linkage.transfer(),
                lift.transfer(),
                lift.waitUntilFinished(0,150),
                lift.findZero()
        );
    }


    public Action safePositions(){
        return  new ParallelAction(
                ptolLatch.unlock(),
                ptorLatch.unlock(),
                sweeper.retracted(),
                outArm.middle(),
                claw.close(),
                inPivot.transfer(),
                inArm.transfer(),
                extendo.findZero(),
                lift.findZero()

        );
    }

    public Action intakeTransfer(){
        return new SequentialAction(
                inPivot.transfer(),
                inArm.transfer(),
                extendo.retracted(),
                extendo.waitUntilFinished(0, 50),
                extendo.findZero()
        );
    }

    public Action grabUntilSpecimen(Enums.DetectedColor allianceColor) {
        return new ParallelAction(
                new SequentialAction(
                        lift.transfer(),
                        inArm.specimenGrab(),
                        inPivot.specimenGrab(),
                        spin.eatUntil(allianceColor, mc),
                        intakeTransfer()
                ),
                outtakeTransfer()
        );
    }

    public Action grabUntilSample(Enums.DetectedColor allianceColor) {
        return new ParallelAction(
                new SequentialAction(
                        lift.transfer(),
                        inArm.sampleGrab(),
                        inPivot.sampleGrab(),
                        spin.eatUntil(allianceColor, mc),
                        intakeTransfer()
                ),
                outtakeTransfer()
        );
    }

    public Action sampleExtend(double Position) {
        return new SequentialAction(
                inArm.sampleExtended(),
                inPivot.sampleExtended(),
                extendo.set(Position),
                extendo.waitUntilFinished());
    }

    public Action specimenExtend(double Position) {
        return new SequentialAction(
                inArm.specimenExtended(),
                inPivot.specimenExtended(),
                extendo.set(Position),
                extendo.waitUntilFinished());
    }

    public Action outtakeSample() {
        return new SequentialAction(
                spin.eat(),
                claw.partialClose(),
                new SleepAction(0.1),
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.1),
                                outArm.flip(),
                                new SleepAction(0.4),
                                spin.slowpoop(),
                                claw.close(),
                                outArm.middle(),
                                linkage.transfer()
                        ),
                        lift.sample(),
                        inArm.sampleExtended(),
                        inPivot.sampleExtended()
                ),
                lift.waitUntilFinished(),
                outArm.sampleScore()

        );
    }

    public Action outtakeSampleAuto() {
        return new SequentialAction(
                spin.eat(),
                claw.partialClose(),
                new SleepAction(0.1),
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.4),
                                spin.slowpoop(),
                                outArm.middle(),
                                linkage.transfer()
                        ),
                        lift.sample(),
                        inArm.specimenExtended(),
                        inPivot.specimenExtended()
                ),
                lift.waitUntilFinished(),
                outArm.sampleScore()

        );
    }


    public Action intakeSpecimen(){
        return new SequentialAction(
                lift.transfer(),
                lift.waitUntilFinished(),
                lift.findZero(),
                claw.open(),
                outArm.specimenIntake(),
                linkage.retracted()
        );
    }

    public Action outtakeSpecimen(){
        return new ParallelAction(
                new SequentialAction(
                        claw.close(),
                        new SleepAction(0.2),
                        lift.specimen(),
                        outArm.specimenDeposit(),
                        linkage.extended()
                ),
                intakeTransfer()
        );
    }

    public Action depositSpecimen(){
        return new SequentialAction(
                claw.open(),
                new SleepAction(0.1),
                outArm.middle(),
                linkage.retracted(),
                lift.transfer(),
                lift.waitUntilFinished(),
                lift.findZero()
        );
    }


    public Action spitSample(){
        return new SequentialAction(
          inArm.sampleSpit(),
          inPivot.sampleSpit(),
          intakeSpecimen(),
          new SleepAction(0.5),
                spin.poop()
        );
    }



    /* -------------------------------------------------------------------- */
    /* === UTILITY SUB‑CLASSES =========================================== */

    // -------------------------- Extendo ---------------------------------
    public class Extendo {
        public Action set(double pos) {
            return t -> {
                mc.extendo.setTargetPosition(pos);
                return false;
            }; }

        public Action retracted() { return set(0); }
        public Action extended()  { return set(600); }

        public Action waitUntilFinished() {
            return new Action() {
                @Override public boolean run(@NonNull TelemetryPacket t) {
                    return !mc.extendo.closeEnough();
                } };
        }
        public Action findZero() {
            return new SequentialAction(t -> {mc.extendo.findZero();return false;},
                    new ActionHelpers.WaitUntilAction(() -> !mc.extendo.isResetting()));
        }
        public Action waitUntilFinished(double position) {
            return new Action() {
                @Override public boolean run(@NonNull TelemetryPacket t) {return !mc.extendo.closeEnough(position);
                } };
        }

        public Action waitUntilFinished(double position, double range) {
            return new Action() {
                @Override public boolean run(@NonNull TelemetryPacket t) {return !mc.extendo.closeEnough(position, range);
                } };
        }
    }

    // --------------------------- Lift -----------------------------------
    public class Lift {
        public Action set(double pos) { return t -> { mc.lift.setTargetPosition(pos); return false; }; }
        public Action transfer()      { return set(0); }
        public Action specimen()      { return set(350); }
        public Action sample()      { return set(800); }

        public Action findZero() {
            return new SequentialAction(t -> {mc.lift.findZero();return false;},
                    new ActionHelpers.WaitUntilAction(() -> !mc.lift.isResetting()));
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override public boolean run(@NonNull TelemetryPacket t) { return !mc.lift.closeEnough(); }
            }; }

        public Action waitUntilFinished(double position) {
            return new Action() {
                @Override public boolean run(@NonNull TelemetryPacket t) { return !mc.lift.closeEnough(position); }
            }; }

        public Action waitUntilFinished(double position, double range) {
            return new Action() {
                @Override public boolean run(@NonNull TelemetryPacket t) {return !mc.lift.closeEnough(position, range);
                } };
        }
    }


    // --------------------------- Hang -----------------------------------
    public class Hang {
        public Action up()   { return t -> { mc.hangr.setPosition(0); mc.hangl.setPosition(0); return false; }; }
        public Action down() { return t -> { mc.hangr.setPosition( 1); mc.hangl.setPosition( 1); return false; }; }
    }

    /* -------------------------------------------------------------------- */
    /* === NEW SERVO WRAPPER CLASSES ===================================== */

    // ---------------------- Outtake Linkage -----------------------------
    public class OuttakeLinkage {
        private static final double RETRACTED = 0.49;
        private static final double EXTENDED  = 0.98;
        private static final double TRANSFER  = 0.71;
        private Action set(double p) { return t -> { mc.outtakeLinkage.setPosition(p); return false; }; }
        public Action retracted() { return set(RETRACTED); }
        public Action extended()  { return set(EXTENDED);  }
        public Action transfer()  { return set(TRANSFER);  }
    }

    // ------------------------ Outtake Claw ------------------------------
    public class OuttakeClaw {
        private static final double OPEN   = 0.40;
        private static final double TRANS  = 0.30;
        private static final double PART   = 0.17;
        private static final double CLOSED = 0.11;
        private Action set(double p){ return t -> { mc.outtakeClaw.setPosition(p); return false; }; }
        public Action open()          { return set(OPEN);   }
        public Action transfer()      { return set(TRANS);  }
        public Action partialClose()  { return set(PART);   }
        public Action close()         { return set(CLOSED); }
    }

    // ----------------------- Outtake Arm (R+L) --------------------------
    public class OuttakeArm {
        private static final double SPEC_INTAKE   = 0.05;
        private static final double SPEC_DEPOSIT  = 0.82;
        private static final double FLIP          = 0.17;
        private static final double SAMPLE_SCORE  = 0.35;
        private static final double MIDDLE        = 0.50;
        private static final double TRANSFER      = 1.00;
        private Action set(double p){ return t -> { mc.outtakeArmR.setPosition(p); mc.outtakeArmL.setPosition(p); return false; }; }
        public Action specimenIntake()  { return set(SPEC_INTAKE);  }
        public Action specimenDeposit() { return set(SPEC_DEPOSIT); }
        public Action flip()            { return set(FLIP);         }
        public Action sampleScore()     { return set(SAMPLE_SCORE);  }
        public Action middle()          { return set(MIDDLE);        }
        public Action transfer()        { return set(TRANSFER);      }
    }

    // ----------------------- Intake Arm (R+L) ---------------------------
    public class IntakeArm {
        private static final double TRANSFER          = 0.24;
        private static final double SPEC_EXTENDED     = 0.51;
        private static final double SPEC_GRAB         = 0.60;
        private static final double SAMPLE_EXTENDED   = 0.48;
        private static final double SAMPLE_GRAB       = 0.59;
        private static final double SAMPLE_SPIT       = 0.22;
        private Action set(double p){ return t -> { mc.intakeArmR.setPosition(p); mc.intakeArmL.setPosition(p); return false; }; }
        public Action transfer()        { return set(TRANSFER);        }
        public Action specimenExtended(){ return set(SPEC_EXTENDED);   }
        public Action specimenGrab()    { return set(SPEC_GRAB);       }
        public Action sampleExtended()  { return set(SAMPLE_EXTENDED); }
        public Action sampleGrab()      { return set(SAMPLE_GRAB);     }
        public Action sampleSpit()      { return set(SAMPLE_SPIT);     }
    }

    // ----------------------- Intake Pivot ------------------------------
    public class IntakePivot {
        private static final double TRANSFER         = 0.33;
        private static final double SPEC_EXTENDED    = 0.20;
        private static final double SPEC_GRAB        = 0.19;
        private static final double SAMPLE_EXTENDED  = 0.00;
        private static final double SAMPLE_GRAB      = 0.10;
        private static final double SAMPLE_SPIT      = 0.85;
        private Action set(double p){ return t -> { mc.intakePivot.setPosition(p); return false; }; }
        public Action transfer()        { return set(TRANSFER);        }
        public Action specimenExtended(){ return set(SPEC_EXTENDED);   }
        public Action specimenGrab()    { return set(SPEC_GRAB);       }
        public Action sampleExtended()  { return set(SAMPLE_EXTENDED); }
        public Action sampleGrab()      { return set(SAMPLE_GRAB);     }
        public Action sampleSpit()      { return set(SAMPLE_SPIT);     }
    }

    // ------------------------ PTOL latch -------------------------------
    public class PtolLatch {
        private static final double UNLOCK = 0.44;
        private static final double LOCK   = 0.55;
        private Action set(double p) { return t -> { mc.ptol.setPosition(p); return false; }; }
        public Action unlock() { return set(UNLOCK); }
        public Action lock()   { return set(LOCK);   }
    }

    // ------------------------ PTOR latch -------------------------------
    public class PtorLatch {
        private static final double UNLOCK = 0.60;
        private static final double LOCK   = 0.45;
        private Action set(double p) { return t -> { mc.ptor.setPosition(p); return false; }; }
        public Action unlock() { return set(UNLOCK); }
        public Action lock()   { return set(LOCK);   }
    }

    public class Sweeper {
        private static final double EXTENDED = 0.25;
        private static final double RETRACTED   = 0.67;
        private Action set(double p) { return t -> { mc.sweeper.setPosition(p); return false; }; }
        public Action extended() { return set(EXTENDED); }
        public Action retracted()   { return set(RETRACTED);   }
    }



    // --------------------------- Spin -----------------------------------
    public class Spin {
        private final MotorControl motorControl;


        public Spin(MotorControl motorControl) {
            this.motorControl = motorControl;
        }


        public Action eatUntil(Enums.DetectedColor allianceColor, MotorControl motorControl) {
            final Enums.IntakeState[] currentState = {Enums.IntakeState.SEARCHING};
            final boolean[] started = {false};
            // Add a variable to store when the rejecting state was entered
            final long[] rejectingStartTime = {0};
            // Define the delay in milliseconds (adjust as needed)
            final long REJECT_DELAY_MS = 400;

            return telemetryPacket -> {
                if (!started[0]) {
                    started[0] = true;
                    motorControl.spin.setPower(1); // Start spinning forward
                }

                // Read the sensor
                Enums.DetectedColor color = motorControl.getDetectedColor();

                // If yellow is detected, task is finished
                if (color == Enums.DetectedColor.YELLOW ) {
                    motorControl.spin.setPower(1); // Stop motor
                    return false; // Action complete
                }

                boolean correctColorSeen = false;
                if (allianceColor == Enums.DetectedColor.RED) {
                    // Accept RED or YELLOW
                    correctColorSeen = (color == Enums.DetectedColor.RED || color == Enums.DetectedColor.YELLOW);
                } else if (allianceColor == Enums.DetectedColor.BLUE) {
                    // Accept BLUE or YELLOW
                    correctColorSeen = (color == Enums.DetectedColor.BLUE || color == Enums.DetectedColor.YELLOW);
                }

                switch (currentState[0]) {
                    case SEARCHING:
                        if (correctColorSeen) {
                            motorControl.spin.setPower(1);
                            return false; // Action complete
                        }
                        // If an unexpected (rejectable) color is seen, transition to REJECTING
                        if (color != Enums.DetectedColor.BLACK
                                && color != Enums.DetectedColor.UNKNOWN
                                && !correctColorSeen) {
                            currentState[0] = Enums.IntakeState.REJECTING;
                            // Record the time when rejecting starts
                            rejectingStartTime[0] = System.currentTimeMillis();
                            motorControl.spin.setPower(-1); // Spin backward
                        }
                        return true; // Continue searching

                    case REJECTING:
                        long now = System.currentTimeMillis();
                        // Only check for resuming forward spinning after the delay
                        if (now - rejectingStartTime[0] >= REJECT_DELAY_MS) {
                            if (color == Enums.DetectedColor.BLACK || color == Enums.DetectedColor.UNKNOWN) {
                                // Done rejecting; resume forward spinning
                                motorControl.spin.setPower(1);
                                currentState[0] = Enums.IntakeState.SEARCHING;
                                // Reset the timer (optional cleanup)
                                rejectingStartTime[0] = 0;
                            }
                        }
                        return true; // Continue in the REJECTING state until conditions are met
                }

                // Default (should not be reached)
                return false;
            };

        }


        /**
         * Simple "spin forward" action that runs indefinitely unless you remove it.
         * You can adapt the return value to end immediately if desired.
         */
        public Action slow() {
            return telemetryPacket -> {
                motorControl.spin.setPower(0.5);
                return false;
            };
        }


        public Action slowpoop() {
            return telemetryPacket -> {
                motorControl.spin.setPower(-0.45);
                return false;
            };
        }

        public Action eat() {
            return telemetryPacket -> {
                motorControl.spin.setPower(1);
                return false;
            };
        }

        /**
         * Simple "spin backward" action that runs indefinitely.
         */
        public Action poop() {
            return telemetryPacket -> {
                motorControl.spin.setPower(-1.0);
                return false;
            };
        }

        /**
         * Stop the spin motor (one-shot).
         */
        public Action stop() {
            return telemetryPacket -> {
                motorControl.spin.setPower(0.0);
                return false; // done immediately
            };
        }

    }

}
