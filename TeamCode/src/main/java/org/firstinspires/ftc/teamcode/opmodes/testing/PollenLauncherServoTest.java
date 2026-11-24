package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Pollen Launcher Servo Test", group = "Testing")
public class PollenLauncherServoTest extends OpMode {

    CRServo pollenLauncherServo;

    @Override
    public void init() {
        pollenLauncherServo = hardwareMap.get(CRServo.class, "pollenLauncherServo");

        pollenLauncherServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        pollenLauncherServo.setPower(gamepad1.left_stick_x);
    }
}
