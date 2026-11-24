package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;


@TeleOp(name = "Pollen Gate Servo Test", group = "Testing")
public class PollenGateServoTest extends ModularOpMode {

    private TransferSubsystem transferSubsystem;

    @Override
    protected void preload() {
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "pollenGateServo");
        transferSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper, () -> gamepad1.left_bumper);

        registerSubsystem(transferSubsystem);
    }

    @Override
    public void loop() {
        super.loop();
    }
}
