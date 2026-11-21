package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.devices.TurretAngleSource;

/**
 * Bot 1.5 Pollen turret: an Axon servo in CONTINUOUS mode turns the turret through a 40T -> 80T gear. The servo only
 * takes a power, so position comes from a {@link TurretAngleSource} and this class closes the loop.
 *
 * Angle convention: 0 = Pollen pointing straight out the robot's right side, + = counterclockwise seen from above.
 *
 * Homing: the angle source only measures relative motion, so on power-up the turret must find the Hall sensor (a
 * magnet on the turret passing a REV magnetic limit switch on the frame). It always approaches from the same side,
 * slowly, for a repeatable trip point: if it starts on the magnet it first backs off, then seeks. When the sensor
 * trips, that spot is homeAngleDeg. Homing gives up after homingTimeoutSec (FAILED) and stops the servo.
 *
 * Once homed, setTargetDeg() holds an angle: power = kP * error + kS (outside the tolerance), capped at maxPower.
 * Targets are clamped to minAngleDeg..maxAngleDeg, and the servo is never driven further past either limit, so the
 * turret can't wind up its wires. Every Panels value below is live.
 */
@Configurable
public class TurretSubsystem extends Subsystem {

    // Closed loop
    public static double kP = 0.02;              // power per degree of error
    public static double kS = 0.06;              // minimum power that actually moves the turret
    public static double maxPower = 0.8;
    public static double toleranceDeg = 1.0;     // inside this the servo is left at 0 and isOnTarget() is true

    // Travel (turret degrees). Set from Turret Calibration: just short of where wires or hard stops bite.
    public static double minAngleDeg = -90;
    public static double maxAngleDeg = 90;

    // Homing
    public static double homeAngleDeg = 0;       // turret angle where the Hall sensor trips (approaching as below)
    public static double homingPower = 0.15;     // slow, for a repeatable trip point
    public static int homingDirection = -1;      // +1 seeks counterclockwise, -1 clockwise
    public static double homingTimeoutSec = 4.0;
    public static boolean hallActiveLow = true;  // REV magnetic limit switch reads LOW with the magnet present

    // Flip if positive power turns the turret clockwise (seen from above).
    public static boolean servoReversed = false;

    public enum State { NOT_HOMED, HOMING_BACKOFF, HOMING_SEEK, READY, FAILED, MANUAL }

    private final CRServo servo;
    private final DigitalChannel homeSensor;
    private final TurretAngleSource angle;

    private State state = State.NOT_HOMED;
    private final ElapsedTime homingTimer = new ElapsedTime();
    private double targetDeg = 0;
    private double manualPower = 0;
    private double power = 0;

    /**
     * @param servoName      the Axon (configured as a Continuous Rotation Servo)
     * @param homeSensorName the Hall sensor (Digital Device); may be missing from the configuration
     * @param angle          where the turret angle comes from
     */
    public TurretSubsystem(HardwareMap hardwareMap, String servoName, String homeSensorName, TurretAngleSource angle) {
        servo = hardwareMap.tryGet(CRServo.class, servoName);
        homeSensor = hardwareMap.tryGet(DigitalChannel.class, homeSensorName);
        if (homeSensor != null) homeSensor.setMode(DigitalChannel.Mode.INPUT);
        this.angle = angle;
    }

    // ---- Commands ----

    /** Find the Hall sensor (re-homes even if already homed). */
    public void startHoming() {
        if (servo == null || !angle.isConnected() || homeSensor == null) {
            state = State.FAILED;
            return;
        }
        angle.clearHome();
        state = isAtHomeSensor() ? State.HOMING_BACKOFF : State.HOMING_SEEK;
        homingTimer.reset();
    }

    /** Home only if not homed yet (e.g. carried over from Autonomous). */
    public void homeIfNeeded() {
        if (!angle.isHomed() && state != State.HOMING_BACKOFF && state != State.HOMING_SEEK) startHoming();
    }

    /** Hold this turret angle (clamped to the travel limits). Ignored until homed. */
    public void setTargetDeg(double deg) {
        targetDeg = clamp(deg, minAngleDeg, maxAngleDeg);
        if (angle.isHomed() && (state == State.MANUAL || state == State.NOT_HOMED)) state = State.READY;
    }

    /** Open-loop power for calibration, + = counterclockwise. Still stops at the travel limits once homed. */
    public void setManualPower(double p) {
        manualPower = clamp(p, -1, 1);
        if (state != State.HOMING_BACKOFF && state != State.HOMING_SEEK) state = State.MANUAL;
    }

    /** For calibration without a Hall sensor: call the current spot homeAngleDeg. */
    public void setHomeHere() {
        angle.setHome(homeAngleDeg);
        targetDeg = clamp(homeAngleDeg, minAngleDeg, maxAngleDeg);
        if (state != State.MANUAL) state = State.READY;
    }

    // ---- Status ----

    public State getState() { return state; }
    public boolean isHomed() { return angle.isHomed(); }
    public boolean isHoming() { return state == State.HOMING_BACKOFF || state == State.HOMING_SEEK; }
    public double getAngleDeg() { return angle.isHomed() ? angle.getAngleDeg() : Double.NaN; }
    public double getTargetDeg() { return targetDeg; }
    public double getErrorDeg() { return targetDeg - getAngleDeg(); }
    public double getPower() { return power; }
    public boolean isHomeSensorConfigured() { return homeSensor != null; }

    /** Homed, holding a target, and within toleranceDeg of it. */
    public boolean isOnTarget() {
        return state == State.READY && angle.isHomed() && Math.abs(getErrorDeg()) <= toleranceDeg;
    }

    public boolean isAtHomeSensor() {
        return homeSensor != null && (homeSensor.getState() != hallActiveLow);
    }

    // ---- Loop ----

    @Override
    public void init() {
        targetDeg = clamp(homeAngleDeg, minAngleDeg, maxAngleDeg);
        setServo(0);
    }

    @Override
    public void init_loop() {
        angle.update();
        setServo(0);
    }

    @Override
    public void start() { }

    @Override
    public void update() {
        angle.update();
        if (state == State.NOT_HOMED && angle.isHomed()) state = State.READY;   // carried over from an earlier OpMode

        double p = 0;
        switch (state) {
            case HOMING_BACKOFF:
                if (homingTimer.seconds() > homingTimeoutSec) { state = State.FAILED; break; }
                if (!isAtHomeSensor()) state = State.HOMING_SEEK;
                else p = -homingDirection * homingPower;
                break;
            case HOMING_SEEK:
                if (homingTimer.seconds() > homingTimeoutSec) { state = State.FAILED; break; }
                if (isAtHomeSensor()) {
                    angle.setHome(homeAngleDeg);
                    targetDeg = clamp(homeAngleDeg, minAngleDeg, maxAngleDeg);
                    state = State.READY;
                } else {
                    p = homingDirection * homingPower;
                }
                break;
            case READY: {
                double e = getErrorDeg();
                if (Math.abs(e) > toleranceDeg) {
                    p = clamp(kP * e + Math.signum(e) * kS, -maxPower, maxPower);
                }
                break;
            }
            case MANUAL:
                p = manualPower;
                break;
            default:   // NOT_HOMED, FAILED
                break;
        }

        // Never drive further past a travel limit (homing excepted: the angle is unknown then).
        if (angle.isHomed() && !isHoming()) {
            double a = angle.getAngleDeg();
            if ((a >= maxAngleDeg && p > 0) || (a <= minAngleDeg && p < 0)) p = 0;
        }
        setServo(p);
    }

    @Override
    public void stop() { setServo(0); }

    private void setServo(double p) {
        power = p;
        if (servo != null) servo.setPower(servoReversed ? -p : p);
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    public void addTelemetry(Telemetry telemetry) {
        if (servo == null) { telemetry.addLine("TURRET: servo not in configuration"); return; }
        if (!angle.isConnected()) { telemetry.addLine("TURRET: angle sensor not in configuration"); return; }
        String where = angle.isHomed()
                ? String.format("%.1f deg -> %.1f%s", angle.getAngleDeg(), targetDeg, isOnTarget() ? " ON TARGET" : "")
                : "not homed";
        telemetry.addData("Turret", "%s  [%s]", where, state);
    }
}
