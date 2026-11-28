package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.devices.AxonTurretAngleSource;
import org.firstinspires.ftc.teamcode.devices.FusedTurretAngleSource;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Tests the coarse/fine turret angle fusion ({@link FusedTurretAngleSource}): the Through Bore encoder on
 * frontRightDriveMotor's port gives the coarse half-turn, the Axon feedback ("pollenLauncherServoVoltage") the fine angle.
 *
 * Drive the turret by hand with gp1 left stick X and watch the fusion find the location. gp1 X zeros the current spot
 * (homes the coarse encoder). The fine angle also needs {@link AxonTurretAngleSource#servoDegAtZero} set in code -
 * until then it runs coarse-only, which is a safe way to bring it up.
 *
 * What to check: point Pollen straight out the right side, press X, then jog around. "Fused" should track smoothly and
 * agree with "Coarse"; "Fine" should match modulo one sector (180 deg at 2:1). If "Fused" jumps by ~180 the coarse is
 * picking the wrong half - check gearRatio and the home. The angle counting the wrong way means reversed is flipped.
 */
@Configurable
@TeleOp(name = "Turret Fusion Test", group = "Testing")
public class TurretFusionTest extends ModularOpMode {

    public static double manualScale = 0.4;   // stick -> power while jogging

    private FusedTurretAngleSource angle;
    private TurretSubsystem turret;
    private int logCounter = 0;

    @Override
    protected void preload() {
        angle = new FusedTurretAngleSource(hardwareMap, "frontRightDriveMotor", "pollenLauncherServoVoltage");
        turret = new TurretSubsystem(hardwareMap, "pollenLauncherServo", angle);
        registerSubsystem(turret);
    }

    @Override
    public void loop() {
        double stick = gamepad1.left_stick_x;

        if (gamepad1.xWasPressed()) turret.setHomeHere();   // point Pollen straight out the right side first
        if (Math.abs(stick) > 0.05) {
            turret.setManualPower(stick * manualScale);
        } else {
            turret.setTargetDeg(TurretSubsystem.homeAngleDeg);   // release stick -> hold home via the closed loop (shake test)
        }

        super.loop();   // turret.update() runs here

        telemetry.addData("Homed", !turret.isHomed() ? "no"
                : (angle.getCoarse().isCarriedOver() ? "yes (carried over)" : "yes"));
        telemetry.addData("Fused angle", "%.2f deg", turret.getAngleDeg());
        telemetry.addData("Coarse (encoder)", "%.2f deg", angle.getCoarseAngleDeg());
        telemetry.addData("Fine (Axon, mod sector)", "%.2f deg", angle.getFineTurretDeg());
        telemetry.addData("Encoder counts", "%.0f", angle.getCoarse().getCounts());
        telemetry.addData("Axon connected", angle.getFine().isConnected());
        telemetry.addData("Axon voltage", "%.3f V", angle.getFine().getVoltage());
        telemetry.addData("Axon servoDeg (raw)", "%.1f", angle.getFine().getServoDeg());
        telemetry.addData("servoDegAtZero", "%.1f  (NaN until you press X after warm-up)", AxonTurretAngleSource.servoDegAtZero);
        double offsetForZeroHere = FusedTurretAngleSource.zeroOffsetDeg - turret.getAngleDeg();
        telemetry.addData(">> zeroOffsetDeg for 0 here <<", "%.2f  (turn to true zero, then paste this into the code)", offsetForZeroHere);
        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addLine("gp1: stick X jog | X zero here (wait ~3s warm-up first)");
        telemetry.update();

        String diag = String.format("conn=%b v=%.4f servoDeg=%.1f warmS=%.1f homed=%b fine=%.2f coarse=%.2f fused=%.2f z0=%.1f offsetForZero=%.2f",
                angle.getFine().isConnected(), angle.getFine().getVoltage(), angle.getFine().getServoDeg(),
                getRuntime(), turret.isHomed(), angle.getFineTurretDeg(), angle.getCoarseAngleDeg(),
                turret.getAngleDeg(), AxonTurretAngleSource.servoDegAtZero, offsetForZeroHere);
        Log.i("FusionTest", diag);
        if (++logCounter % 10 == 0) {
            try (java.io.FileWriter fw = new java.io.FileWriter("/sdcard/FIRST/fusion_debug.txt")) {
                fw.write(diag + "\n");
            } catch (Exception ignored) {}
        }
    }
}
