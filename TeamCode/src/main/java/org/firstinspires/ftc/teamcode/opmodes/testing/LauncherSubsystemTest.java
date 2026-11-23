package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

/**
 * Bot 1.5 launcher-only test: the single Pollen launcher motor ("pollenLauncherMotor"), nothing else - no transfer,
 * drive or pose. The wheel runs at LauncherSubsystem's fixed default speed plus the dpad offset.
 *
 * gp1 RB: flywheel on/off. gp1 dpad up/down: target speed +/-20 ticks/s. gp1 B: clear the offset.
 * Gains (kS, kV, kP, kI) are live in Panels under LauncherSubsystem; the velocity graph is on Panels too.
 */
@TeleOp(name = "Launcher Pollen only Subsystem Test", group = "Testing")
public class LauncherSubsystemTest extends ModularOpMode {

    private LauncherSubsystem launcherSubsystem;
    private boolean wheelOn = false;

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor");
        launcherSubsystem.setBindings(bindingManager,
                () -> wheelOn,
                () -> gamepad1.dpadUpWasPressed(),
                () -> gamepad1.dpadDownWasPressed());
        registerSubsystem(launcherSubsystem);
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) wheelOn = !wheelOn;
        if (gamepad1.bWasPressed()) launcherSubsystem.resetOffsets();

        super.loop();

        double target = launcherSubsystem.getPollenTargetVelocity();
        double actual = launcherSubsystem.getPollenCurrentVelocity();
        telemetry.addData("Flywheel (gp1 RB)", wheelOn ? "ON" : "OFF");
        telemetry.addData("Target / actual", "%.0f / %.0f ticks/s", target, actual);
        telemetry.addData("Error", "%.0f ticks/s", wheelOn ? target - actual : 0.0);
        telemetry.addData("Speed offset (dpad up/down, B clears)", "%+.0f", launcherSubsystem.getPollenOffset());
        telemetry.update();
    }
}
