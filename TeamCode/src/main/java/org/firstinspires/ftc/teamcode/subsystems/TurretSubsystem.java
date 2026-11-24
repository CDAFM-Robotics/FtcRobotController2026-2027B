package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * Angles are continuous (no wrap). The wires allow one full turn: minAngleDeg..maxAngleDeg = -180..180 around home,
 * so the two ends point the same way (the "seam", straight out the robot's LEFT side with home at the right side).
 *
 * Aiming ({@link #setAimDeg}): of the angles that point the requested way (deg + k*360), take the one nearest the
 * current target, allowing up to aimHysteresisDeg past either limit, then clamp to the travel. Following a moving
 * target never jumps; when it crosses the seam the turret holds at the limit (aim off by up to aimHysteresisDeg)
 * until the target is aimHysteresisDeg past it, then UNWINDS the full turn back to the other side (isOnTarget() is
 * false meanwhile, which holds up firing). The hysteresis stops a target sitting on the seam from swinging it back
 * and forth.
 *
 * Start window: the program always starts with the turret in startWindowCenterDeg +/- 90 (bot 1.5: 0..+180), with
 * home at the window's edge. That lets the angle source know the angle at startup (absolute start) and tells
 * homing which way the magnet is.
 *
 * Homing (fallback, e.g. before servoDegAtZero is calibrated; the turret must be in the start window):
 *  1. SEARCH: moves toward home at searchPower, at most across the whole window plus searchMarginDeg, until the
 *     Hall sensor trips;
 *  2. BACKOFF: moves against homingDirection until the sensor clears, then backoffMarginDeg more;
 *  3. SEEK: moves slowly in homingDirection; the trip point is homeAngleDeg. Always the same edge of the detection
 *     zone, approached from the same side at the same slow speed, so it is repeatable.
 * Soft limits are off while homing (the angle is unknown), which is why it only searches toward home and no further.
 * Gives up (FAILED, servo stopped) when the search finds nothing or after homingTimeoutSec.
 *
 * Usually no homing motion is needed: the angle source can know the angle at startup (AxonTurretAngleSource absolute
 * start - the program always starts with the turret in 0..+180 - or a home carried over from Autonomous). Homing
 * with the Hall sensor is the fallback (gp2 X in TeleOp).
 *
 * Hall check: with no hard stop, a wrong angle could wind the wires past their end. The wires allow exactly one turn,
 * so the magnet passes the sensor at only one spot in the whole travel - home. Every time it trips while moving in
 * homingDirection the turret IS at homeAngleDeg; if the angle reads more than hallFaultDeg off (a missed servo turn,
 * an absolute start outside the window), it is corrected on the spot. Smaller differences are only recorded: they
 * are mostly the sensor being read once per loop while the turret moves, and correcting them would add error.
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
    public static double toleranceDeg = 1.0;     // inside this the servo is left at 0 and isOnTarget() is true
    public static double unwindingDeg = 90;      // error above this is reported as UNWINDING
    public static double aimHysteresisDeg = 10;  // how far past the seam the target goes before the turret unwinds

    // Travel (turret degrees): one full turn of wire. Set from Turret Calibration, a little inside where the wires get
    // tight; the directions between the limit and the seam are then reached by holding at the limit (small aim error).
    public static double minAngleDeg = -180;
    public static double maxAngleDeg = 180;

    // Homing
    public static double homeAngleDeg = 0;       // turret angle where the Hall sensor trips (approaching as below)
    public static int homingDirection = -1;      // final approach: +1 counterclockwise, -1 clockwise
    public static double homingPower = 0.15;     // final approach, slow for a repeatable trip point
    public static double searchPower = 0.35;     // search toward home
    public static double searchMarginDeg = 5;    // search goes this far beyond the far edge of the start window
    public static double startWindowCenterDeg = 90;   // the program always starts with the turret within +/- 90 of this
    public static double backoffMarginDeg = 10;  // past the sensor clearing, before the final approach
    public static double homingTimeoutSec = 12.0;
    public static boolean hallActiveLow = true;  // REV magnetic limit switch reads LOW with the magnet present
    public static double hallFaultDeg = 20;      // Hall check: errors bigger than this at the magnet get corrected

    // Flip if positive power turns the turret clockwise (seen from above).
    public static boolean servoReversed = false;

    public enum State { NOT_HOMED, HOMING_SEARCH, HOMING_BACKOFF, HOMING_SEEK, READY, FAILED, MANUAL }

    private final CRServo servo;
    private final DigitalChannel homeSensor;
    private final TurretAngleSource angle;

    private State state = State.NOT_HOMED;
    private final ElapsedTime homingTimer = new ElapsedTime();
    private double searchStartDeg = 0;
    private double backoffClearDeg = Double.NaN;
    private double targetDeg = 0;
    private double manualPower = 0;
    private double power = 0;
    private boolean prevHall = false;
    private double lastHallCheckErrDeg = Double.NaN;
    private int hallCorrections = 0;

    /**
     * @param servoName      the Axon (configured as a Continuous Rotation Servo)
     * @param homeSensorName the Hall sensor (Digital Device); may be missing from the configuration
     * @param angle          where the turret angle comes from
     */
    public TurretSubsystem(HardwareMap hardwareMap, String servoName, String homeSensorName, TurretAngleSource angle) {
        servo = hardwareMap.get(CRServo.class, servoName);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        homeSensor = hardwareMap.tryGet(DigitalChannel.class, homeSensorName);
        if (homeSensor != null) homeSensor.setMode(DigitalChannel.Mode.INPUT);
        this.angle = angle;
    }

    // ---- Commands ----

    /** Find the Hall sensor (re-homes even if already homed). The turret must be in the start window. */
    public void startHoming() {
        if (servo == null || !angle.isConnected() || homeSensor == null) {
            state = State.FAILED;
            return;
        }
        angle.clearHome();
        homingTimer.reset();
        if (isAtHomeSensor()) {
            backoffClearDeg = Double.NaN;
            state = State.HOMING_BACKOFF;
        } else {
            searchStartDeg = angle.getAngleDeg();
            state = State.HOMING_SEARCH;
        }
    }

    /** Home only if not homed yet (e.g. carried over from Autonomous). */
    public void homeIfNeeded() {
        if (!angle.isHomed() && !isHoming()) startHoming();
    }

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
        if (!isHoming()) state = State.MANUAL;
    }

    /** For calibration: call the current spot homeAngleDeg (no Hall sensor needed). */
    public void setHomeHere() {
        angle.setHome(homeAngleDeg);
        targetDeg = clamp(homeAngleDeg, minAngleDeg, maxAngleDeg);
        if (state != State.MANUAL) state = State.READY;
    }

    // ---- Status ----

    public State getState() { return state; }
    public boolean isHomed() { return angle.isHomed(); }
    public boolean isHoming() {
        return state == State.HOMING_SEARCH || state == State.HOMING_BACKOFF || state == State.HOMING_SEEK;
    }
    public double getAngleDeg() { return angle.isHomed() ? angle.getAngleDeg() : Double.NaN; }
    public double getTargetDeg() { return targetDeg; }
    public double getErrorDeg() { return targetDeg - getAngleDeg(); }
    public double getPower() { return power; }
    public boolean isHomeSensorConfigured() { return homeSensor != null; }
    /** Angle error at the last Hall check (deg), NaN before the first. */
    public double getLastHallCheckErrDeg() { return lastHallCheckErrDeg; }
    /** How many times the Hall check has corrected the angle this OpMode. */
    public int getHallCorrections() { return hallCorrections; }

    /** Homed, holding a target, and within toleranceDeg of it. */
    public boolean isOnTarget() {
        return state == State.READY && angle.isHomed() && Math.abs(getErrorDeg()) <= toleranceDeg;
    }

    /** Swinging the long way round to the other equivalent angle. */
    public boolean isUnwinding() {
        return state == State.READY && angle.isHomed() && Math.abs(getErrorDeg()) > unwindingDeg;
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

    /** Search move relative to the start: toward home, as far as home can be from anywhere in the start window. */
    private double searchOffset() {
        double dir = Math.signum(homeAngleDeg - startWindowCenterDeg);
        if (dir == 0) dir = homingDirection;
        return dir * (Math.abs(homeAngleDeg - startWindowCenterDeg) + 90 + searchMarginDeg);
    }

    private double updateHoming() {
        if (homingTimer.seconds() > homingTimeoutSec) { state = State.FAILED; return 0; }
        double a = angle.getAngleDeg();   // relative only, until homed
        switch (state) {
            case HOMING_SEARCH: {
                if (isAtHomeSensor()) {
                    backoffClearDeg = Double.NaN;
                    state = State.HOMING_BACKOFF;
                    return 0;
                }
                double off = searchOffset();
                double end = searchStartDeg + off;
                if ((off > 0 && a >= end) || (off < 0 && a <= end)) { state = State.FAILED; return 0; }   // no magnet
                return Math.signum(off) * searchPower;
            }
            case HOMING_BACKOFF:
                if (Double.isNaN(backoffClearDeg)) {
                    if (!isAtHomeSensor()) backoffClearDeg = a;   // just cleared: now go the margin further
                } else if (Math.abs(a - backoffClearDeg) >= backoffMarginDeg) {
                    state = State.HOMING_SEEK;
                    return 0;
                }
                return -homingDirection * homingPower;
            case HOMING_SEEK:
                if (isAtHomeSensor()) {
                    angle.setHome(homeAngleDeg);
                    targetDeg = clamp(homeAngleDeg, minAngleDeg, maxAngleDeg);
                    state = State.READY;
                    return 0;
                }
                return homingDirection * homingPower;
            default:
                return 0;
        }
    }

    /** See class comment. Runs before this loop's power is chosen; power still holds last loop's command. */
    private void hallCheck() {
        boolean hall = isAtHomeSensor();
        if (hall && !prevHall && angle.isHomed() && !isHoming() && Math.signum(power) == homingDirection) {
            lastHallCheckErrDeg = angle.getAngleDeg() - homeAngleDeg;
            if (Math.abs(lastHallCheckErrDeg) > hallFaultDeg) {
                angle.setHome(homeAngleDeg);   // the magnet is only at home: trust it over the count
                hallCorrections++;
            }
        }
        prevHall = hall;
    }

    @Override
    public void update() {
        angle.update();
        if (state == State.NOT_HOMED && angle.isHomed()) state = State.READY;   // absolute start / earlier OpMode
        hallCheck();

        double p = 0;
        switch (state) {
            case HOMING_SEARCH:
            case HOMING_BACKOFF:
            case HOMING_SEEK:
                p = updateHoming();
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
        String where;
        if (!angle.isHomed()) where = state == State.FAILED ? "HOMING FAILED (turn into 0..+180, gp2 X)" : "not homed";
        else where = String.format("%.1f deg -> %.1f%s", angle.getAngleDeg(), targetDeg,
                isOnTarget() ? " ON TARGET" : (isUnwinding() ? " UNWINDING" : ""));
        telemetry.addData("Turret", "%s  [%s]", where, state);
        if (hallCorrections > 0) {
            telemetry.addData("Turret Hall corrections", "%d (last %.0f deg) - check the feedback wire", hallCorrections, lastHallCheckErrDeg);
        }
    }
}
