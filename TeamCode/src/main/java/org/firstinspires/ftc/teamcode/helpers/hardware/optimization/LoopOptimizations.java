package org.firstinspires.ftc.teamcode.helpers.hardware.optimization;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

public final class LoopOptimizations {
    private LoopOptimizations() { }

    public static final class HardwareWriteCache {
        private static final double DEFAULT_MOTOR_EPSILON = 0.002;
        private static final double DEFAULT_SERVO_EPSILON = 0.003;

        private static final Map<DcMotor, Double> LAST_MOTOR_POWER = new IdentityHashMap<>();
        private static final Map<Servo, Double> LAST_SERVO_POSITION = new IdentityHashMap<>();

        private HardwareWriteCache() { }

        public static void clear() {
            LAST_MOTOR_POWER.clear();
            LAST_SERVO_POSITION.clear();
        }

        public static void setMotorPower(DcMotor motor, double power) {
            setMotorPower(motor, power, DEFAULT_MOTOR_EPSILON);
        }

        public static void setMotorPower(DcMotor motor, double power, double epsilon) {
            if (motor == null) return;
            Double last = LAST_MOTOR_POWER.get(motor);
            if (last == null || Math.abs(last - power) > epsilon) {
                motor.setPower(power);
                LAST_MOTOR_POWER.put(motor, power);
            }
        }

        public static void setServoPosition(Servo servo, double position) {
            setServoPosition(servo, position, DEFAULT_SERVO_EPSILON);
        }

        public static void setServoPosition(Servo servo, double position, double epsilon) {
            if (servo == null) return;
            Double last = LAST_SERVO_POSITION.get(servo);
            if (last == null || Math.abs(last - position) > epsilon) {
                servo.setPosition(position);
                LAST_SERVO_POSITION.put(servo, position);
            }
        }
    }

    public static final class BulkCacheManager {
        private final List<LynxModule> hubs;

        public BulkCacheManager(HardwareMap hardwareMap) {
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }

        public void clear() {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
        }
    }

    public static final class TelemetryThrottler {
        private final long minIntervalNs;
        private long lastUpdateNs = 0;

        public TelemetryThrottler(double hz) {
            double clampedHz = Math.max(1.0, hz);
            minIntervalNs = (long) (1_000_000_000.0 / clampedHz);
        }

        public boolean shouldUpdate() {
            long now = System.nanoTime();
            if (now - lastUpdateNs >= minIntervalNs) {
                lastUpdateNs = now;
                return true;
            }
            return false;
        }
    }
}
