package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.actuators.ServoDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Configurable
public class IntakeSubsystem extends Subsystem {

    // --- Flower bar ---
    // A goBILDA Speed servo ("wedgeServo", Control Hub servo port 5) swings a bar down to push Pollen out of the
    // Flower so the intake can pick them up. Optional: if the servo is not in the configuration, Flower pickup does
    // nothing and the rest of the intake works as before. Higher position = further down; the up hard stop is near 0.30.
    public static final double POSITION_SERVO_WEDGE_UP = 0.15;   // measured Nov 14 with WedgeServoTest
    public static final double POSITION_SERVO_WEDGE_DOWN = 0.5;   // measured Nov 14
    public static int wedgeMoveMs = 400;    // down -> up travel; extender stays in this long after a raise
    public static double flowerIntakePower = 1.0;    // intake power while the bar is down (0.75 felt slow, Nov 14)
    public static double flowerIntakeStopDelaySec = 0.5; // intake keeps running this long after the raise starts

    private final MotorDevice intakeMotor;
    private final ServoDevice leftExtenderServo;
    private final ServoDevice rightExtenderServo;

    // Axon feedback — null if not wired
    private final AnalogInput leftFeedback;
    private final AnalogInput rightFeedback;

    // Extender feedback voltages, measured Nov 14 with ExtenderFeedbackCalibration (old left values were 1.885 / 2.196).
    // INIT for reference: left 1.670, right 1.635. Left rises as the extender goes out; right falls (mirrored servo).
    // Resting noise is about +/-0.004 V; readings spike while the servos move, hence the hold time below.
    public static double extLeftRetractedV  = 1.717;
    public static double extLeftExtendedV   = 2.163;
    public static double extRightRetractedV = 1.568;
    public static double extRightExtendedV  = 1.130;
    // One-sided thresholds: "retracted" = at RETRACTED or further in (INIT counts), within this margin;
    // "extended" = at EXTENDED or further out, within this margin.
    public static double extFeedbackMarginV = 0.03;
    // A side's reading must satisfy the threshold continuously this long before it counts (filters motion spikes).
    public static int    extFeedbackHoldMs  = 100;
    // Feedback-corrected retract: while the extender should be retracted and has finished its ramp, any side whose
    // feedback is outside the retracted range has its servo command moved this much further in per loop, never past
    // INIT. Balls pushing the extender out (e.g. during Flower pickup) are pulled back this way. Reset on extend.
    public static boolean retractCorrectionEnabled = true;
    public static double  retractCorrectionStep    = 0.002;

    private BindingManager bindingManager;
    private final Telemetry telemetry;
    private boolean useAnalogIntake = false;

    // Servo position constants — DO NOT CHANGE

    // Left Servo: Positive to extend

    public static final double POSITION_EXTENSION_SERVO_INIT_LEFT = 0.48;
    public static final double POSITION_EXTENSION_SERVO_RETRACTED_LEFT = 0.505;
    public static final double POSITION_EXTENSION_SERVO_EXTENDED_LEFT = 0.585;

    // Right Servo: Negative to extend

    public static final double POSITION_EXTENSION_SERVO_INIT_RIGHT = 0.485;
    public static final double POSITION_EXTENSION_SERVO_RETRACTED_RIGHT = 0.46;
    public static final double POSITION_EXTENSION_SERVO_EXTENDED_RIGHT = 0.38;

    // Tolerance for the no-feedback "at retracted/extended" position estimate.
    private static final double POSITION_TOLERANCE = 0.05;

    private double currentLeftPos  = POSITION_EXTENSION_SERVO_INIT_LEFT;
    private double currentRightPos = POSITION_EXTENSION_SERVO_INIT_RIGHT;

    // Debounced extender state, computed once per update() from both feedback signals.
    private boolean retractedStable = false;
    private boolean extendedStable  = false;
    private final ElapsedTime retractedRawSince = new ElapsedTime();
    private final ElapsedTime extendedRawSince  = new ElapsedTime();
    private boolean retractedRawPrev = false;
    private boolean extendedRawPrev  = false;
    // Per-side "in retracted range" from this loop's raw reading (true when that side has no feedback).
    private boolean leftInRetractedRange  = true;
    private boolean rightInRetractedRange = true;
    // Retract correction, as a distance further in than RETRACTED (0 .. |RETRACTED - INIT|), per side.
    private double retractCorrL = 0;
    private double retractCorrR = 0;

    private final ServoDevice wedgeServo;            // null if not configured
    private boolean wedgeDown = false;         // bar commanded down
    private boolean flowerRaisedOnce = false;      // a raise has happened (so the stop-delay tail can run)
    private final ElapsedTime sinceFlowerRaise = new ElapsedTime();

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String intakeMotorName, String leftExtenderServoName, String rightExtenderServoName, String wedgeServoName) {
        intakeMotor = new MotorDevice(hardwareMap, intakeMotorName);
        leftExtenderServo = new ServoDevice(hardwareMap, leftExtenderServoName);
        rightExtenderServo = new ServoDevice(hardwareMap, rightExtenderServoName);
        wedgeServo = new ServoDevice(hardwareMap, wedgeServoName);
        leftFeedback  = hardwareMap.tryGet(AnalogInput.class, "intakeLeftServoSignal");
        rightFeedback = hardwareMap.tryGet(AnalogInput.class, "intakeRightServoSignal");
        
        this.telemetry = telemetry;
    }

    public boolean isWedgeConfigured() { return wedgeServo != null; }
    public boolean isWedgeDown()       { return wedgeDown; }

    /** Flower pickup requested but the bar is still up, waiting for the extender to retract. */
    public boolean isFlowerWaitingForRetract() {
        return bindingManager.checkBinding("flowerPickup") && !wedgeDown && wedgeServo != null;
    }

    /** Bar is up and has had wedgeMoveMs to get there since the last raise. */
    public boolean isWedgeUp() {
        return !wedgeDown && (!flowerRaisedOnce || sinceFlowerRaise.milliseconds() >= wedgeMoveMs);
    }

    public void setBindings(BindingManager bindingManager, BooleanSupplier intake, BooleanSupplier extender, BooleanSupplier flowerPickup, BooleanSupplier initExtender) {
        bindingManager.addBinding("intake", intake);
        bindingManager.addBinding("extendIntake", extender);
        bindingManager.addBinding("initExtender", initExtender);
        bindingManager.addBinding("flowerPickup", flowerPickup);
        this.bindingManager = bindingManager;
    }

    public void setBindings(BindingManager bindingManager, DoubleSupplier intakeSpeed, BooleanSupplier extender, BooleanSupplier flowerPickup, BooleanSupplier initExtender) {
        bindingManager.addAnalog("intakeSpeed", intakeSpeed);
        bindingManager.addBinding("extendIntake", extender);
        bindingManager.addBinding("initExtender", initExtender);
        bindingManager.addBinding("flowerPickup", flowerPickup);
        this.bindingManager = bindingManager;
        useAnalogIntake = true;
    }

    // --- Feedback accessors ---

    public double getLeftFeedbackVoltage() {
        return leftFeedback != null ? leftFeedback.getVoltage() : Double.NaN;
    }

    public double getRightFeedbackVoltage() {
        return rightFeedback != null ? rightFeedback.getVoltage() : Double.NaN;
    }

    /** Extender at EXTENDED or further out: every wired feedback side agrees and has held for extFeedbackHoldMs.
     *  Updated once per update(); false before the first update(). */
    public boolean isExtended() {
        return extendedStable;
    }

    /** Extender at RETRACTED or further in (INIT counts): every wired feedback side agrees and has held for
     *  extFeedbackHoldMs. Updated once per update(); false before the first update(). */
    public boolean isRetracted() {
        return retractedStable;
    }

    /** Current retract correction per side, in servo units further in than RETRACTED (0 = none). */
    public double getRetractCorrectionLeft()  { return retractCorrL; }
    public double getRetractCorrectionRight() { return retractCorrR; }

    /** A side has been pulled in all the way to INIT and its feedback is still outside the retracted range. */
    public boolean isRetractCorrectionAtLimit() {
        double maxL = Math.abs(POSITION_EXTENSION_SERVO_RETRACTED_LEFT  - POSITION_EXTENSION_SERVO_INIT_LEFT);
        double maxR = Math.abs(POSITION_EXTENSION_SERVO_RETRACTED_RIGHT - POSITION_EXTENSION_SERVO_INIT_RIGHT);
        return (retractCorrL >= maxL - 1e-9 && !leftInRetractedRange)
            || (retractCorrR >= maxR - 1e-9 && !rightInRetractedRange);
    }

    /** True if voltage v is at 'end' or beyond it (away from 'other'), allowing margin. Works for rising or falling sides. */
    private static boolean atOrBeyond(double v, double end, double other, double margin) {
        double outward = Math.signum(end - other);   // direction from the other end toward this end
        return (v - end) * outward >= -margin;
    }

    /** Reads both feedback signals once and updates the debounced retracted/extended state. */
    private void updateExtenderFeedback() {
        double vL = leftFeedback  != null ? leftFeedback.getVoltage()  : Double.NaN;
        double vR = rightFeedback != null ? rightFeedback.getVoltage() : Double.NaN;

        leftInRetractedRange  = leftFeedback  == null || atOrBeyond(vL, extLeftRetractedV,  extLeftExtendedV,  extFeedbackMarginV);
        rightInRetractedRange = rightFeedback == null || atOrBeyond(vR, extRightRetractedV, extRightExtendedV, extFeedbackMarginV);

        boolean retractedRaw, extendedRaw;
        if (leftFeedback == null && rightFeedback == null) {
            // No feedback wired: fall back to the commanded position estimate.
            retractedRaw = Math.abs(currentLeftPos - POSITION_EXTENSION_SERVO_RETRACTED_LEFT) < POSITION_TOLERANCE / 2;
            extendedRaw  = Math.abs(currentLeftPos - POSITION_EXTENSION_SERVO_EXTENDED_LEFT)  < POSITION_TOLERANCE / 2;
        } else {
            retractedRaw = (leftFeedback  == null || atOrBeyond(vL, extLeftRetractedV,  extLeftExtendedV,   extFeedbackMarginV))
                        && (rightFeedback == null || atOrBeyond(vR, extRightRetractedV, extRightExtendedV,  extFeedbackMarginV));
            extendedRaw  = (leftFeedback  == null || atOrBeyond(vL, extLeftExtendedV,   extLeftRetractedV,  extFeedbackMarginV))
                        && (rightFeedback == null || atOrBeyond(vR, extRightExtendedV,  extRightRetractedV, extFeedbackMarginV));
        }

        if (retractedRaw && !retractedRawPrev) retractedRawSince.reset();
        if (extendedRaw  && !extendedRawPrev)  extendedRawSince.reset();
        retractedRawPrev = retractedRaw;
        extendedRawPrev  = extendedRaw;
        retractedStable = retractedRaw && retractedRawSince.milliseconds() >= extFeedbackHoldMs;
        extendedStable  = extendedRaw  && extendedRawSince.milliseconds()  >= extFeedbackHoldMs;
    }

    // --- Subsystem lifecycle ---

    @Override
    public void init() {
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentLeftPos  = POSITION_EXTENSION_SERVO_INIT_LEFT;
        currentRightPos = POSITION_EXTENSION_SERVO_INIT_RIGHT;
        leftExtenderServo.setPosition(currentLeftPos);
        rightExtenderServo.setPosition(currentRightPos);
        leftExtenderServo.update();
        rightExtenderServo.update();
        // Bar up at init.
        wedgeDown = false;
        flowerRaisedOnce = false;
        wedgeServo.setPosition(POSITION_SERVO_WEDGE_UP);
        wedgeServo.update();
    }

    @Override public void init_loop() {}
    @Override public void start() {}

    @Override
    public void update() {
        updateExtenderFeedback();

        // Flower bar: lower only once the extender is retracted; raise as soon as pickup is no longer requested.
        boolean flowerWanted = bindingManager.checkBinding("flowerPickup");
        if (flowerWanted && !wedgeDown && wedgeServo != null && isRetracted()) {
            wedgeDown = true;
        } else if (!flowerWanted && wedgeDown) {
            wedgeDown = false;
            flowerRaisedOnce = true;
            sinceFlowerRaise.reset();
        }

            wedgeServo.setPosition(wedgeDown ? POSITION_SERVO_WEDGE_DOWN : POSITION_SERVO_WEDGE_UP);

        boolean flowerIntakeOn = wedgeDown
                || (flowerRaisedOnce && sinceFlowerRaise.seconds() < flowerIntakeStopDelaySec);

        // Intake motor: the driver's binding wins whenever it asks for anything; otherwise Flower pickup runs it.
        double requested;
        if (useAnalogIntake) {
            requested = bindingManager.checkAnalog("intakeSpeed");
        } else {
            requested = bindingManager.checkBinding("intake") ? 1 : 0;
        }
        intakeMotor.setPower(requested != 0 ? requested : (flowerIntakeOn ? flowerIntakePower : 0));

        // Extender target. Interlock: the extender stays retracted while Flower pickup is requested
        // (this is what retracts it before the bar lowers), while the bar is down, and while the bar is still rising.
        // initExtender overrides everything and parks both servos at their INIT position.
        boolean initRequested = bindingManager.checkBinding("initExtender");
        boolean extend = bindingManager.checkBinding("extendIntake")
                && !flowerWanted && isWedgeUp() && !initRequested;
        if (extend || initRequested) {
            retractCorrL = 0;
            retractCorrR = 0;
        }
        // "Further in" direction per side is from RETRACTED toward INIT.
        double inL = Math.signum(POSITION_EXTENSION_SERVO_INIT_LEFT  - POSITION_EXTENSION_SERVO_RETRACTED_LEFT);
        double inR = Math.signum(POSITION_EXTENSION_SERVO_INIT_RIGHT - POSITION_EXTENSION_SERVO_RETRACTED_RIGHT);
        double leftGoal, rightGoal;
        if (initRequested) {
            leftGoal  = POSITION_EXTENSION_SERVO_INIT_LEFT;
            rightGoal = POSITION_EXTENSION_SERVO_INIT_RIGHT;
        } else {
            leftGoal  = extend
                    ? POSITION_EXTENSION_SERVO_EXTENDED_LEFT
                    : POSITION_EXTENSION_SERVO_RETRACTED_LEFT + inL * retractCorrL;
            rightGoal = extend
                    ? POSITION_EXTENSION_SERVO_EXTENDED_RIGHT
                    : POSITION_EXTENSION_SERVO_RETRACTED_RIGHT + inR * retractCorrR;
        }

        currentLeftPos  = leftGoal;
        currentRightPos = rightGoal;

        // Feedback-corrected retract: while a side should be retracted, pull it further in (for the next loop) while its
        // feedback is outside the retracted range, up to INIT.
        if (!extend && !initRequested && retractCorrectionEnabled) {
            double maxL = Math.abs(POSITION_EXTENSION_SERVO_RETRACTED_LEFT  - POSITION_EXTENSION_SERVO_INIT_LEFT);
            double maxR = Math.abs(POSITION_EXTENSION_SERVO_RETRACTED_RIGHT - POSITION_EXTENSION_SERVO_INIT_RIGHT);
            if (leftFeedback != null && !leftInRetractedRange) {
                retractCorrL = Math.min(maxL, retractCorrL + retractCorrectionStep);
            }
            if (rightFeedback != null && !rightInRetractedRange) {
                retractCorrR = Math.min(maxR, retractCorrR + retractCorrectionStep);
            }
        }

        leftExtenderServo.setPosition(currentLeftPos);
        rightExtenderServo.setPosition(currentRightPos);

        intakeMotor.update();
        leftExtenderServo.update();
        rightExtenderServo.update();
        wedgeServo.update();
        LoopProfiler.mark("intake");
    }

    @Override public void stop() {}
}
