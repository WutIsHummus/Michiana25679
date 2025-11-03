package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotActions {
    
    private final DcMotor intakefront;
    private final DcMotor intakeback;
    private final DcMotor shootr;
    private final DcMotor shootl;
    private final Servo launchgate;
    private final Servo reargate;
    
    public final IntakeFront intakeFront;
    public final IntakeBack intakeBack;
    public final Shooter shooter;
    public final LaunchGate launch;
    public final RearGate rear;
    
    public RobotActions(DcMotor intakefront, DcMotor intakeback, DcMotor shootr, DcMotor shootl, 
                        Servo launchgate, Servo reargate) {
        this.intakefront = intakefront;
        this.intakeback = intakeback;
        this.shootr = shootr;
        this.shootl = shootl;
        this.launchgate = launchgate;
        this.reargate = reargate;
        
        intakeFront = new IntakeFront();
        intakeBack = new IntakeBack();
        shooter = new Shooter();
        launch = new LaunchGate();
        rear = new RearGate();
    }
    
    public Action startIntake() {
        return new ParallelAction(
                intakeFront.run(),
                intakeBack.run()
        );
    }
    
    public Action stopIntake() {
        return new ParallelAction(
                intakeFront.stop(),
                intakeBack.stop()
        );
    }
    
    public Action shootSequence() {
        return new SequentialAction(
                shooter.spinUp(),
                new SleepAction(0.5),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset()
        );
    }
    
    public Action rapidFire() {
        return new SequentialAction(
                shooter.spinUp(),
                intakeFront.run(),
                intakeBack.run(),
                new SleepAction(0.5),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                new SleepAction(0.3),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                shooter.stop()
        );
    }
    
    public Action intakeAndLaunch() {
        return new SequentialAction(
                intakeBack.run(),
                intakeFront.run(),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                new SleepAction(0.2),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                new SleepAction(0.2),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset()
        );
    }
    
    public Action safePositions() {
        return new ParallelAction(
                intakeFront.stop(),
                intakeBack.stop(),
                shooter.stop(),
                launch.reset(),
                rear.close()
        );
    }
    
    public class IntakeFront {
        public Action run() {
            return new InstantAction(() -> intakefront.setPower(-1.0));
        }
        
        public Action runSlow() {
            return new InstantAction(() -> intakefront.setPower(-0.5));
        }
        
        public Action reverse() {
            return new InstantAction(() -> intakefront.setPower(1.0));
        }
        
        public Action stop() {
            return new InstantAction(() -> intakefront.setPower(0.0));
        }
    }
    
    public class IntakeBack {
        public Action run() {
            return new InstantAction(() -> intakeback.setPower(-1.0));
        }
        
        public Action runSlow() {
            return new InstantAction(() -> intakeback.setPower(-0.5));
        }
        
        public Action reverse() {
            return new InstantAction(() -> intakeback.setPower(1.0));
        }
        
        public Action stop() {
            return new InstantAction(() -> intakeback.setPower(0.0));
        }
    }
    
    public class Shooter {
        public Action spinUp() {
            return new InstantAction(() -> {
                shootr.setPower(1.0);
                shootl.setPower(-1.0);
            });
        }
        
        public Action spinUpSlow() {
            return new InstantAction(() -> {
                shootr.setPower(0.7);
                shootl.setPower(-0.7);
            });
        }
        
        public Action stop() {
            return new InstantAction(() -> {
                shootr.setPower(0.0);
                shootl.setPower(0.0);
            });
        }
    }
    
    public class LaunchGate {
        public Action fire() {
            return new InstantAction(() -> launchgate.setPosition(0.8));
        }
        
        public Action reset() {
            return new InstantAction(() -> launchgate.setPosition(0.5));
        }
        
        public Action open() {
            return new InstantAction(() -> launchgate.setPosition(1.0));
        }
        
        public Action close() {
            return new InstantAction(() -> launchgate.setPosition(0.0));
        }
    }
    
    public class RearGate {
        public Action open() {
            return new InstantAction(() -> reargate.setPosition(1.0));
        }
        
        public Action close() {
            return new InstantAction(() -> reargate.setPosition(0.0));
        }
        
        public Action middle() {
            return new InstantAction(() -> reargate.setPosition(0.5));
        }
    }
}

