package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.devices.actuators.ServoDevice;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Intake Extender Tuner Test", group = "Testing")
public class IntakeExtenderTunerTest extends OpMode {

    private ServoImplEx leftExtenderServo;
    private ServoImplEx rightExtenderServo;

    @Override
    public void init() {
        leftExtenderServo = hardwareMap.get(ServoImplEx.class, "leftExtenderServo");
        rightExtenderServo = hardwareMap.get(ServoImplEx.class, "rightExtenderServo");
    }

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    private boolean leftServoActive = true;

    private double leftServoPos = 0.5;
    private double rightServoPos = 0.5;

    @Override
    public void loop() {
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.a && !previousGamepad1.a) {
            leftServoActive = !leftServoActive;
        }

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            if (leftServoActive) {
                leftServoPos += 0.05;
            }
            else {
                rightServoPos += 0.05;
            }
        }

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            if (leftServoActive) {
                leftServoPos -= 0.05;
            }
            else {
                rightServoPos -= 0.05;
            }
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            if (leftServoActive) {
                leftServoPos += 0.005;
            }
            else {
                rightServoPos += 0.005;
            }
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            if (leftServoActive) {
                leftServoPos -= 0.005;
            }
            else {
                rightServoPos -= 0.005;
            }
        }



        leftExtenderServo.setPosition(leftServoPos);
        rightExtenderServo.setPosition(rightServoPos);

        telemetry.addData("Active Servo", leftServoActive ? "Left Extender Servo" : "Right Extender Servo");
        telemetry.addData("Left Servo Position", leftServoPos);
        telemetry.addData("Right Servo Position", rightServoPos);
        telemetry.update();

        previousGamepad1.copy(currentGamepad1);
    }
}
