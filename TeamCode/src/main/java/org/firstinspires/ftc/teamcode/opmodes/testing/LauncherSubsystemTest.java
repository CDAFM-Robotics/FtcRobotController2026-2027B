package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * Bot 1.5 launch test with the single Pollen launcher motor ("pollenLauncherMotor") and the transfer. No drive, no
 * pose: the wheel runs at LauncherSubsystem's fixed default speed plus the dpad offset.
 *
 * gp1 RB: toggle the flywheel. gp1 RT (hold): launch - the transfer feeds only while the wheel is within readyBandTicks
 * of its target, same gate as Competition TeleOp, so each ball waits for the wheel to recover. While the wheel spins
 * without launching, the transfer's auto-nudge is held so balls aren't pushed into a slow wheel.
 * gp1 dpad up/down: target speed +/-20 ticks/s. gp1 B: clear the offset.
 */
@Configurable
@TeleOp(name = "Launcher Subsystem Test", group = "Testing")
public class LauncherSubsystemTest extends ModularOpMode {

    public static double readyBandTicks = 40;

    private LauncherSubsystem launcherSubsystem;
    private TransferSubsystem transferSubsystem;

    private boolean wheelOn = false;

    private boolean launching() { return gamepad1.right_trigger > 0.1; }

    /** Wheel spins while toggled on or while launching. */
    private boolean wheelEnabled() { return wheelOn || launching(); }

    private boolean wheelReady() {
        return Math.abs(launcherSubsystem.getPollenCurrentVelocity())
                >= launcherSubsystem.getPollenTargetVelocity() - readyBandTicks;
    }

    /** Feed only while launching with the wheel at speed. */
    private boolean feeding() { return launching() && wheelReady(); }

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor");
        launcherSubsystem.setBindings(bindingManager,
                this::wheelEnabled,
                () -> gamepad1.dpadUpWasPressed(),
                () -> gamepad1.dpadDownWasPressed());
        registerSubsystem(launcherSubsystem);

        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        transferSubsystem.setBindings(bindingManager, this::feeding, this::feeding);
        transferSubsystem.setFeedHold(() -> wheelEnabled() && !feeding());
        registerSubsystem(transferSubsystem);
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) wheelOn = !wheelOn;
        if (gamepad1.bWasPressed()) launcherSubsystem.resetOffsets();

        super.loop();

        telemetry.addData("Flywheel (gp1 RB)", "%s  %.0f / %.0f ticks/s", wheelEnabled() ? "ON" : "OFF",
                launcherSubsystem.getPollenCurrentVelocity(), launcherSubsystem.getPollenTargetVelocity());
        telemetry.addData("Speed offset (dpad up/down, B clears)", "%+.0f", launcherSubsystem.getPollenOffset());
        telemetry.addData("Launch (hold gp1 RT)", feeding() ? "FEEDING" : (launching() ? "waiting for wheel" : "idle"));
        telemetry.addData("Transfer mode", transferSubsystem.getMode());
        telemetry.update();
    }
}
