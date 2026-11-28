package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Diagnostic: reads the Axon feedback analog input ("pollenLauncherServoVoltage") and both shows it on telemetry and
 * logs it to logcat (tag "AnalogProbe"), so the raw value can be read over adb while the turret is turned.
 *
 * Also grabs and drives the turret servo ("pollenLauncherServo") - the Axon's feedback is powered from the servo
 * port's 5V rail, so this guarantees the servo is initialised and energised while we read. gp1 left stick X jogs it.
 */
@TeleOp(name = "Analog Voltage Probe", group = "Testing")
public class AnalogVoltageProbe extends LinearOpMode {

    @Override
    public void runOpMode() {
        AnalogInput axon = hardwareMap.tryGet(AnalogInput.class, "pollenLauncherServoVoltage");
        CRServo servo = hardwareMap.tryGet(CRServo.class, "pollenLauncherServo");
        if (servo != null) servo.setPower(0);   // initialise / energise the servo port

        Log.i("AnalogProbe", "axon=" + (axon == null ? "NULL" : "FOUND") + "  servo=" + (servo == null ? "NULL" : "FOUND"));
        telemetry.addData("axon input", axon == null ? "NULL - not in config" : "found");
        telemetry.addData("servo", servo == null ? "NULL - not in config" : "found");
        if (axon != null) telemetry.addData("maxVoltage", "%.3f V", axon.getMaxVoltage());
        telemetry.addLine("START, then gp1 left stick X to turn the turret (or turn by hand) and watch the voltage.");
        telemetry.update();

        waitForStart();

        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        while (opModeIsActive()) {
            double power = gamepad1.left_stick_x * 0.3;
            if (servo != null) servo.setPower(power);

            double v = axon == null ? Double.NaN : axon.getVoltage();
            if (!Double.isNaN(v)) { min = Math.min(min, v); max = Math.max(max, v); }

            telemetry.addData("pollenLauncherServoVoltage", "%.4f V", v);
            telemetry.addData("seen min / max", "%.4f / %.4f V", min, max);
            telemetry.addData("servo power", "%.2f", power);
            telemetry.update();
            Log.i("AnalogProbe", String.format("v=%.4f  min=%.4f  max=%.4f  power=%.2f", v, min, max, power));
            sleep(50);
        }
    }
}
