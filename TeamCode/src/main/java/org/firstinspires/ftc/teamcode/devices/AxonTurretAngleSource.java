package org.firstinspires.ftc.teamcode.devices;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Turret angle from the Axon servo's analog feedback wire.
 *
 * The wire gives the SERVO shaft angle within one turn (minVoltage .. maxVoltage = 0 .. 360 deg). The servo drives the
 * turret through a 40T -> 80T gear (gearRatio = 2), so one turret turn is two servo turns and the raw reading repeats
 * every 180 deg of turret. We therefore track the servo continuously: each loop adds the change since the last loop,
 * wrapped to -180..180, to a running total. That stays correct as long as the servo moves less than 180 deg between
 * two loops (an Axon at full speed does ~10 deg per 20 ms loop, so only a long stall could break it).
 *
 * The total only means something relative to a known point. At the first reading the home comes from, in order:
 *  1. the previous OpMode (Autonomous -> TeleOp): kept in static fields, valid if the turret moved less than 90 deg
 *     (180 servo deg) in between - normally it does not move at all;
 *  2. ABSOLUTE START: the program always starts with the turret inside one 180 deg window of travel
 *     (TurretSubsystem.startWindowCenterDeg +/- 90; bot 1.5 starts in 0..+180). 180 turret deg is exactly one servo turn, so inside
 *     that window each reading belongs to exactly one turret angle. Needs servoDegAtZero (the servo reading with the
 *     turret at 0), shown by Turret Calibration once homed. The turret must start inside the window for this to be right;
 *  3. otherwise not homed, until a home is set (e.g. Turret Calibration's "zero here").
 *
 * Accuracy: this measures the servo, so backlash between the 40T and 80T gears shows up as aim error. A Through Bore
 * encoder on the turret itself removes that.
 */
@Configurable
public class AxonTurretAngleSource implements TurretAngleSource {

    // Feedback voltage at 0 and 360 servo degrees. The Axon nominally spans 0 - 3.3 V but never quite reaches the ends;
    // spin the turret in Turret Calibration and copy the min/max it reports here.
    public static double minVoltage = 0.0;
    public static double maxVoltage = 3.3;
    // Servo turns per turret turn (80T turret gear / 40T servo gear).
    public static double gearRatio = 2.0;
    // Flip if the turret angle goes DOWN while the turret turns counterclockwise (seen from above). Set true from the
    // fusion data (Nov 27): the Axon fine angle ran opposite to the Through Bore coarse (fine ≈ -coarse).
    public static boolean reversed = true;
    // Absolute start (see class comment). servoDegAtZero = the servo reading (0..360) at turret mechanical zero; it is
    // the fixed offset between the Axon's electrical zero and mechanical zero, so the absolute reading gives the turret
    // angle with no homing. Measured Nov 27 (servo zero sits ~43 deg from mechanical zero). NaN turns absolute start off.
    public static double servoDegAtZero = 86.3;

    // The Axon's analog feedback is invalid (reads garbage / near 0) for the first few seconds after the servo port is
    // powered. Ignore it until this long after construction, so the baseline / absolute start is taken from a real
    // reading instead of warm-up garbage.
    public static double warmupSec = 2.5;

    // Seam-glitch guard: crossing the 0/3.3 V wrap, the Axon feedback does not jump cleanly - it lingers at a mid-range
    // voltage (~1.0-1.5 V) for a couple of loops before settling, which looks like a huge angle step. Reject any
    // per-loop servo change beyond maxServoStepDeg and HOLD the last good reading, so the next clean sample telescopes
    // the running total correctly. The threshold sits above real servo speed (~10 deg/loop) and below the glitch
    // (~100+ deg/loop). After maxFeedbackSkips held loops in a row, trust the reading again so a genuine fast move
    // cannot desync the count forever.
    public static double maxServoStepDeg  = 60;
    public static int    maxFeedbackSkips = 8;

    // Carried across OpModes (see class comment).
    private static boolean storedHomed = false;
    private static double storedServoTotalDeg = 0;
    private static double storedHomeServoTotalDeg = 0;
    private static double storedHomeTurretDeg = 0;

    private final AnalogInput signal;

    private boolean initialized = false;
    private double lastServoDeg = 0;     // 0..360, last loop's raw reading
    private double servoTotalDeg = 0;    // running total, unbounded
    private boolean homed = false;
    private double homeServoTotalDeg = 0;
    private double homeTurretDeg = 0;
    private boolean carriedOver = false;
    private boolean absoluteStart = false;
    private double voltage = Double.NaN;
    private int skips = 0;               // consecutive glitch samples held (see maxServoStepDeg)
    private final ElapsedTime warmup = new ElapsedTime();   // since construction (≈ servo power-up)

    public AxonTurretAngleSource(HardwareMap hardwareMap, String signalName) {
        signal = hardwareMap.tryGet(AnalogInput.class, signalName);
    }

    @Override
    public boolean isConnected() { return signal != null; }

    /** Raw feedback voltage this loop (NaN when not connected). For calibration. */
    public double getVoltage() { return voltage; }

    /** Servo shaft angle within one turn, 0..360, this loop. For calibration. */
    public double getServoDeg() { return lastServoDeg; }

    /** Servo angle including full turns since power-up / the stored value. For calibration. */
    public double getServoTotalDeg() { return servoTotalDeg; }

    /**
     * Turret angle implied by this loop's raw servo reading, correct MODULO one servo turn (360/gearRatio turret deg
     * = one "sector"). Absolute within a sector via servoDegAtZero, but ambiguous between sectors - a coarse sensor
     * (e.g. the Through Bore encoder) picks the sector. Uses the last trusted reading, so it holds through a seam
     * glitch rather than jumping. NaN if the Axon is not connected or servoDegAtZero is not set.
     */
    public double getFineTurretDeg() {
        if (signal == null || !initialized || Double.isNaN(servoDegAtZero)) return Double.NaN;
        double sign = reversed ? -1 : 1;
        return sign * (lastServoDeg - servoDegAtZero) / gearRatio;
    }

    /**
     * Calibrate servoDegAtZero so this loop's raw reading maps to {@code turretDegNow} (mod one sector). Lets a home
     * from another sensor (e.g. the fused coarse encoder) auto-anchor the fine angle, instead of needing servoDegAtZero
     * typed in. No-op until the Axon is connected and warmed up (otherwise the reading is garbage).
     */
    public void calibrateFineZero(double turretDegNow) {
        if (signal == null || !initialized) return;
        double sign = reversed ? -1 : 1;
        double z = (lastServoDeg - sign * gearRatio * turretDegNow) % 360.0;
        servoDegAtZero = z < 0 ? z + 360.0 : z;
    }

    /** True when the home came from an earlier OpMode rather than homing in this one. */
    public boolean isCarriedOver() { return carriedOver; }

    /** True when the angle came from the absolute start (no homing motion) and has not been re-homed since. */
    public boolean isAbsoluteStart() { return absoluteStart; }

    /** servoDegAtZero implied by the current home (0..360): copy it into the code. NaN until homed. */
    public double getServoDegAtZeroEstimate() {
        if (!homed) return Double.NaN;
        double sign = reversed ? -1 : 1;
        double z = (servoTotalDeg - sign * gearRatio * getAngleDeg()) % 360.0;
        return z < 0 ? z + 360.0 : z;
    }

    /** Turret angle for this servo reading, inside the start window (one servo turn = 360 / gearRatio turret deg). */
    private double absoluteAngle(double servoDeg) {
        double sign = reversed ? -1 : 1;
        double width = 360.0 / gearRatio;
        double base = sign * (servoDeg - servoDegAtZero) / gearRatio;
        double lo = TurretSubsystem.startWindowCenterDeg - width / 2;
        double m = (base - lo) % width;
        if (m < 0) m += width;
        return lo + m;
    }

    private double readServoDeg() {
        voltage = signal.getVoltage();
        double span = maxVoltage - minVoltage;
        double frac = span > 0 ? (voltage - minVoltage) / span : 0;
        frac = Math.max(0, Math.min(1, frac));
        return frac * 360.0;
    }

    private static double wrap180(double deg) {
        deg %= 360.0;
        if (deg >= 180) deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }

    @Override
    public void update() {
        if (signal == null) return;
        double now = readServoDeg();   // updates voltage for telemetry even during warm-up
        if (warmup.seconds() < warmupSec) return;   // analog not valid yet - don't baseline / accumulate from it
        if (!initialized) {
            initialized = true;
            if (storedHomed) {
                // Pick the running total nearest to where the last OpMode left it that matches this reading.
                servoTotalDeg = storedServoTotalDeg + wrap180(now - storedServoTotalDeg);
                homeServoTotalDeg = storedHomeServoTotalDeg;
                homeTurretDeg = storedHomeTurretDeg;
                homed = true;
                carriedOver = true;
            } else {
                servoTotalDeg = now;
                if (!Double.isNaN(servoDegAtZero)) {
                    setHome(absoluteAngle(now));
                    absoluteStart = true;
                }
            }
            lastServoDeg = now;
        } else {
            double d = wrap180(now - lastServoDeg);
            if (Math.abs(d) <= maxServoStepDeg || skips >= maxFeedbackSkips) {
                // Normal motion, the real seam wrap (folds to a small step), or a resync after too many held loops.
                servoTotalDeg += d;
                lastServoDeg = now;
                skips = 0;
            } else {
                // Seam glitch: ignore this mid-range sample and hold the last good reading (see maxServoStepDeg).
                skips++;
            }
        }
        storedServoTotalDeg = servoTotalDeg;
    }

    @Override
    public boolean isHomed() { return homed; }

    @Override
    public double getAngleDeg() {
        double sign = reversed ? -1 : 1;
        return homeTurretDeg + sign * (servoTotalDeg - homeServoTotalDeg) / gearRatio;
    }

    @Override
    public void setHome(double turretDeg) {
        homeServoTotalDeg = servoTotalDeg;
        homeTurretDeg = turretDeg;
        homed = true;
        carriedOver = false;
        absoluteStart = false;
        storedHomed = true;
        storedHomeServoTotalDeg = homeServoTotalDeg;
        storedHomeTurretDeg = homeTurretDeg;
    }

    @Override
    public void clearHome() {
        homed = false;
        carriedOver = false;
        absoluteStart = false;
        storedHomed = false;
    }
}
