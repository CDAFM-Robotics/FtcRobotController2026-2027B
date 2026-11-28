package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.devices.TurretAngleSource;

/**
 * Bot 1.5 Pollen turret: an Axon servo in CONTINUOUS mode turns the turret through a 40T -> 80T gear. The servo only
 * takes a power, so position comes from a {@link TurretAngleSource} and this class closes the loop.
 *
 * Angle convention: 0 = mechanical zero, the middle of the cord's travel, + = counterclockwise seen from above.
 * Angles are continuous (no wrap). The wires allow one full turn: minAngleDeg..maxAngleDeg = -180..180 around home,
 * so the two ends point the same way (the "seam", directly opposite mechanical zero).
 *
 * Aiming ({@link #setAimDeg}): of the angles that point the requested way (deg + k*360), take the one nearest the
 * current target, allowing up to aimHysteresisDeg past either limit, then clamp to the travel. Following a moving
 * target never jumps; when it crosses the seam the turret holds at the limit (aim off by up to aimHysteresisDeg)
 * until the target is aimHysteresisDeg past it, then UNWINDS the full turn back to the other side (isOnTarget() is
 * false meanwhile, which holds up firing). The hysteresis stops a target sitting on the seam from swinging it back
 * and forth.
 *
 * Home: the angle comes from the {@link TurretAngleSource}, which knows it at startup via an absolute start
 * (AxonTurretAngleSource.servoDegAtZero, with the turret in startWindowCenterDeg +/- 90) or a home carried over from
 * Autonomous. {@link #setHomeHere} calls the current spot home, for calibration.
 *
 * Once homed: power = kP * error + kS (outside the tolerance), capped at maxPower; the servo is never driven further
 * past a travel limit. Every Panels value below is live.
 */
@Configurable
public class TurretSubsystem extends Subsystem {

    // Closed loop
    public static double kP = 0.02;              // power per degree of error
    public static double kS = 0.06;              // minimum power that actually moves the turret
    public static double maxPower = 0.8;
    public static double toleranceDeg = 1.0;     // inside this the servo holds idleHoldPower and isOnTarget() is true
    public static double idleHoldPower = 0.001;  // tiny power held when on target; well under kS so it never moves the turret
    public static double unwindingDeg = 90;      // error above this is reported as UNWINDING
    public static double aimHysteresisDeg = 10;  // how far past the seam the target goes before the turret unwinds

    // Travel (turret degrees): one full turn of wire. Set from Turret Calibration, a little inside where the wires get
    // tight; the directions between the limit and the seam are then reached by holding at the limit (small aim error).
    public static double minAngleDeg = -180;
    public static double maxAngleDeg = 180;

    // Home reference
    public static double homeAngleDeg = 0;            // reference zero; setHomeHere() calls the current turret spot this
    public static double startWindowCenterDeg = 0;   // the turret must power up within +/- 90 of this (mechanical zero) for the Axon absolute start to pick the right half

    // Flip if positive power turns the turret clockwise (seen from above).
    // Nov 27: set true after auto-aim turned the wrong way (left when it should go right). The whole turret sense was
    // mirrored, so the two sensor signs were flipped with it (ThroughBoreTurretAngleSource.reversed and
    // AxonTurretAngleSource.reversed). All three must flip together: flipping the servo alone makes the loop run away.
    public static boolean servoReversed = true;

    public enum State { NOT_HOMED, READY, MANUAL }

    private final CRServo servo;
    private final TurretAngleSource angle;

    private State state = State.NOT_HOMED;
    private double targetDeg = 0;
    private double manualPower = 0;
    private double power = 0;

    /**
     * @param servoName the Axon (configured as a Continuous Rotation Servo)
     * @param angle     where the turret angle comes from
     */
    public TurretSubsystem(HardwareMap hardwareMap, String servoName, TurretAngleSource angle) {
        servo = hardwareMap.get(CRServo.class, servoName);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        this.angle = angle;
    }

    // ---- Commands ----

    /** Hold this exact turret angle (clamped to the travel). Ignored until homed. */
    public void setTargetDeg(double deg) {
        targetDeg = clamp(deg, minAngleDeg, maxAngleDeg);
        if (angle.isHomed() && (state == State.MANUAL || state == State.NOT_HOMED)) state = State.READY;
    }

    /**
     * Point the turret this way (any angle; deg and deg + k*360 are the same direction). Picks the equivalent angle
     * nearest the current target within the travel widened by aimHysteresisDeg, then clamps, so it holds at a limit
     * across the seam and unwinds only once the target is clearly past it (see class comment).
     */
    public void setAimDeg(double deg) {
        if (Double.isNaN(deg)) return;
        double lo = minAngleDeg - aimHysteresisDeg, hi = maxAngleDeg + aimHysteresisDeg;
        double best = Double.NaN;
        for (int k = -3; k <= 3; k++) {
            double c = deg + 360.0 * k;
            if (c < lo || c > hi) continue;
            if (Double.isNaN(best) || Math.abs(c - targetDeg) < Math.abs(best - targetDeg)) best = c;
        }
        // Travel well under 360 and the direction falls in the gap: get as close as the travel allows.
        if (Double.isNaN(best)) best = deg;
        setTargetDeg(best);   // clamps to minAngleDeg..maxAngleDeg
    }

    /** Open-loop power for calibration, + = counterclockwise. Still stops at the travel limits once homed. */
    public void setManualPower(double p) {
        manualPower = clamp(p, -1, 1);
        state = State.MANUAL;
    }

    /** For calibration: call the current spot homeAngleDeg. */
    public void setHomeHere() {
        angle.setHome(homeAngleDeg);
        targetDeg = clamp(homeAngleDeg, minAngleDeg, maxAngleDeg);
        if (state != State.MANUAL) state = State.READY;
    }

    // ---- Status ----

    public State getState() { return state; }
    public boolean isHomed() { return angle.isHomed(); }
    public double getAngleDeg() { return angle.isHomed() ? angle.getAngleDeg() : Double.NaN; }
    public double getTargetDeg() { return targetDeg; }
    public double getErrorDeg() { return targetDeg - getAngleDeg(); }
    public double getPower() { return power; }

    /** Homed, holding a target, and within toleranceDeg of it. */
    public boolean isOnTarget() {
        return state == State.READY && angle.isHomed() && Math.abs(getErrorDeg()) <= toleranceDeg;
    }

    /** Swinging the long way round to the other equivalent angle. */
    public boolean isUnwinding() {
        return state == State.READY && angle.isHomed() && Math.abs(getErrorDeg()) > unwindingDeg;
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
        if (state == State.NOT_HOMED && angle.isHomed()) state = State.READY;   // absolute start / earlier OpMode

        double p = 0;
        switch (state) {
            case READY: {
                double e = getErrorDeg();
                if (Math.abs(e) > toleranceDeg) {
                    p = clamp(kP * e + Math.signum(e) * kS, -maxPower, maxPower);
                } else {
                    p = idleHoldPower;   // tiny holding power while settled keeps the Axon feedback fresh (bot 2 2025-26)
                }
                break;
            }
            case MANUAL:
                p = manualPower;
                break;
            default:   // NOT_HOMED
                break;
        }

        // Never drive further past a travel limit.
        if (angle.isHomed()) {
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

    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry telemetry) {
        if (servo == null) { telemetry.addLine("TURRET: servo not in configuration"); return; }
        if (!angle.isConnected()) { telemetry.addLine("TURRET: angle sensor not in configuration"); return; }
        String where;
        if (!angle.isHomed()) where = "not homed";
        else where = String.format("%.1f deg -> %.1f%s", angle.getAngleDeg(), targetDeg,
                isOnTarget() ? " ON TARGET" : (isUnwinding() ? " UNWINDING" : ""));
        telemetry.addData("Turret", "%s  [%s]", where, state);
    }
}
