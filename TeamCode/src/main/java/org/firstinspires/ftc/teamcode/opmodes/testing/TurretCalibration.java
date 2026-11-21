package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.devices.AxonTurretAngleSource;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Bring-up and tuning for the bot 1.5 turret. Configuration: "turretServo" (Continuous Rotation Servo),
 * "turretServoSignal" (Analog Input, Axon feedback wire), "turretHomeSensor" (Digital Device, Hall sensor).
 *
 * Suggested order (all values are live in Panels under TurretSubsystem / AxonTurretAngleSource):
 *  1. Direction: gp1 left stick right should turn the turret COUNTERCLOCKWISE seen from above. If not, flip
 *     TurretSubsystem.servoReversed.
 *  2. Feedback range: spin the turret a few full servo turns with the stick; copy "V seen min / max" into
 *     AxonTurretAngleSource.minVoltage / maxVoltage.
 *  3. Angle direction: press X to zero, then stick right - the turret angle must go UP. If not, flip
 *     AxonTurretAngleSource.reversed. Turn the turret 90 deg by hand-measure; the angle should read ~90 (else check gearRatio).
 *  4. Hall sensor: wave the magnet past it; "Hall" must read MAGNET only near it (else flip hallActiveLow).
 *  5. Homing: press A. Set homingDirection so it reaches the magnet the short way, then set homeAngleDeg to the
 *     turret angle at the trip point (0 if the magnet is placed with Pollen pointing straight out the right side).
 *  6. Limits: with the stick, find where wires or hard stops bite; set minAngleDeg / maxAngleDeg a few deg inside.
 *  7. Loop: dpad left/right step the target by stepDeg. Raise kS until small steps move, then kP until it is quick
 *     without overshoot. Y toggles a sweep between -sweepDeg and +sweepDeg to check it repeatably.
 */
@Configurable
@TeleOp(name = "Turret Calibration", group = "Testing")
public class TurretCalibration extends ModularOpMode {

    public static double manualScale = 0.4;   // stick -> power while in manual
    public static double stepDeg = 15;
    public static double sweepDeg = 45;
    public static double sweepPeriodSec = 1.5;

    private AxonTurretAngleSource angle;
    private TurretSubsystem turret;

    private double target = 0;
    private boolean sweeping = false;
    private final com.qualcomm.robotcore.util.ElapsedTime sweepTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private double vMin = Double.POSITIVE_INFINITY, vMax = Double.NEGATIVE_INFINITY;

    @Override
    protected void preload() {
        angle = new AxonTurretAngleSource(hardwareMap, "turretServoSignal");
        turret = new TurretSubsystem(hardwareMap, "turretServo", "turretHomeSensor", angle);
        registerSubsystem(turret);
    }

    @Override
    public void loop() {
        double stick = gamepad1.left_stick_x;

        if (gamepad1.aWasPressed()) { sweeping = false; turret.startHoming(); }
        if (gamepad1.xWasPressed()) { sweeping = false; turret.setHomeHere(); target = TurretSubsystem.homeAngleDeg; }
        if (gamepad1.bWasPressed()) { sweeping = false; turret.setManualPower(0); }
        if (gamepad1.yWasPressed()) { sweeping = !sweeping; sweepTimer.reset(); }
        if (gamepad1.dpadRightWasPressed()) { sweeping = false; target += stepDeg; turret.setTargetDeg(target); }
        if (gamepad1.dpadLeftWasPressed())  { sweeping = false; target -= stepDeg; turret.setTargetDeg(target); }
        target = turret.getTargetDeg();   // reflect clamping

        if (sweeping && turret.isHomed()) {
            boolean high = ((int) (sweepTimer.seconds() / sweepPeriodSec)) % 2 == 0;
            turret.setTargetDeg(high ? sweepDeg : -sweepDeg);
        } else if (Math.abs(stick) > 0.05 && !turret.isHoming()) {
            sweeping = false;
            turret.setManualPower(stick * manualScale);
        } else if (turret.getState() == TurretSubsystem.State.MANUAL) {
            turret.setManualPower(0);
        }

        super.loop();   // turret.update() runs here

        double v = angle.getVoltage();
        if (!Double.isNaN(v)) { vMin = Math.min(vMin, v); vMax = Math.max(vMax, v); }

        turret.addTelemetry(telemetry);
        telemetry.addData("State", turret.getState());
        telemetry.addData("Turret angle / target / error", "%.1f / %.1f / %.1f deg",
                turret.getAngleDeg(), turret.getTargetDeg(), turret.getErrorDeg());
        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addData("Feedback V", "%.3f   (seen min / max %.3f / %.3f)", v, vMin, vMax);
        telemetry.addData("Servo deg in turn / total", "%.1f / %.1f", angle.getServoDeg(), angle.getServoTotalDeg());
        telemetry.addData("Hall", !turret.isHomeSensorConfigured() ? "NOT CONFIGURED (turretHomeSensor)"
                : (turret.isAtHomeSensor() ? "MAGNET" : "clear"));
        telemetry.addData("Homed", turret.isHomed() ? (angle.isCarriedOver() ? "yes (from earlier OpMode)" : "yes") : "no");
        telemetry.addLine("gp1: stick X manual | A home | X zero here | dpad L/R step | Y sweep | B stop");
        telemetry.update();
    }
}
