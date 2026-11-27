package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp(name = "Scoring Subsystems Test", group = "Testing")
public class ScoringSubsystemTest extends ModularOpMode {

    private IntakeSubsystem intakeSubsystem;
    private LauncherSubsystem launcherSubsystem;
    private TransferSubsystem transferSubsystem;

    @Override
    protected void preload() {

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo", "wedgeServo");
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "pollenGateServo");
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor");

        intakeSubsystem.setBindings(bindingManager, () -> gamepad1.b, () -> gamepad1.x, () -> false, () -> gamepad1.y);
        transferSubsystem.setBindings(bindingManager, () -> gamepad1.b || gamepad1.right_trigger > 0, () -> gamepad1.right_trigger > 0);
        launcherSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper || gamepad1.right_trigger > 0);

        registerSubsystem(intakeSubsystem);
        registerSubsystem(transferSubsystem);
        registerSubsystem(launcherSubsystem);
    }
}
