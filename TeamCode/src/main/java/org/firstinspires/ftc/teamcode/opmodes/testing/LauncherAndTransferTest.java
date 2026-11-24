package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp(name = "Launcher and Transfer Test", group = "Testing")
public class LauncherAndTransferTest extends ModularOpMode {

    private LauncherSubsystem launcherSubsystem;
    private TransferSubsystem transferSubsystem;

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor");
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "pollenGateServo");

        launcherSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper);
        transferSubsystem.setBindings(bindingManager, () -> gamepad1.right_trigger > 0, () -> gamepad1.right_trigger > 0);

        registerSubsystem(launcherSubsystem);
        registerSubsystem(transferSubsystem);
    }
}
