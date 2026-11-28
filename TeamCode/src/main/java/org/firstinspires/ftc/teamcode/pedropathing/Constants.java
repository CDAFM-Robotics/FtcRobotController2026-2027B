package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Robot configuration for Pedro Pathing 3.0.
 *
 * <p>Pedro 3 builds a {@link Follower} from three explicit pieces — a drivetrain, a localizer and a
 * following algorithm — instead of the 2.x {@code FollowerBuilder}/{@code FollowerConstants}. Each
 * piece is configured below and assembled in {@link #createFollower(HardwareMap)}.
 *
 * <p>The localizer and Foresight blocks hold tuned values (Pinpoint Tuner and Foresight Tuner, Nov 2026).
 * To re-tune, run the AutoTune procedures registered in {@link Tuning} <b>in this order</b>; each
 * produces the values the next one needs:
 *
 * <ol>
 *   <li><b>Mecanum Tuner</b> → the four {@code *Direction} values in {@link #drivetrainConfig}.
 *   <li><b>Pinpoint Tuner</b> → the pod directions and offsets in {@link #localizerConfig}.
 *   <li><b>Foresight Tuner</b> → the whole of {@link #foresightConfig}.
 *   <li><b>Tests</b> → verifies the result.
 * </ol>
 *
 * <p>Each tuner ends on a page of generated Java. Paste it over the matching block below.
 */
public class Constants {
    /**
     * Drivetrain: four-motor mecanum.
     *
     * <p>Motor names carried over from the previous drive-encoder config. Left side reversed
     * (mirror-mounted motors). Re-run the Mecanum Tuner if a motor is remounted or rewired.
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftDriveMotor");
        c.frontRightName.set("frontRightDriveMotor");
        c.backLeftName.set("backLeftDriveMotor");
        c.backRightName.set("backRightDriveMotor");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    /**
     * Localizer: goBILDA Pinpoint with 4-bar odometry pods.
     *
     * <p>Replaces the 2.x drive-encoder localizer, which Pedro 3 removed. The pod offsets and
     * directions are the Pinpoint Tuner's output for bot 1.5 (tuned Nov 2026). Re-run the tuner and
     * paste its {@code PinpointConfig} block over this one if a pod is moved.
     *
     * <p>{@code LocalizerSubsystem} (TeleOp) reads this same config, so TeleOp and autonomous always
     * configure the Pinpoint identically. Do not copy these numbers anywhere else.
     *
     * <p>{@code name} must match the Pinpoint's name in the Robot Controller configuration.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Nov 27 (late): Localization Test showed both pods counting backwards (forward -> -x, left -> -y), so both
        // directions are REVERSED. The Pinpoint Tuner measured the offsets with the pods still counting backwards,
        // which negates its result, so the offsets below are the tuner's values (-7.449, 2.694) with the signs
        // flipped. Check: spin in place - x and y should stay put. If they swing, re-run the Pinpoint Tuner.
        c.xPodOffset.set(7.44902603269562);
        c.yPodOffset.set(-2.6935222956139273);

        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    /**
     * Following algorithm: Foresight. Tuned with the Foresight Tuner (Nov 2026).
     *
     * <p>These values are properties of this robot's mass, wheels and battery, so they have no
     * sensible defaults: twelve of {@link ForesightConfig}'s variables throw
     * {@code IllegalStateException} when read unset. After a significant mechanical change, re-run
     * the Foresight Tuner and paste its generated block in place of this lambda body.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.31552990869062886);
            Controller secondaryTranslationalForward = Controller.proportional(0.1165798701467491);
            Controller primaryTranslationalLateral = Controller.proportional(0.3968134313803307);
            Controller secondaryTranslationalLateral = Controller.proportional(0.14661195984486658);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.012910126255410138));
            c.brake.set(Controller.proportionalFeedforward(0.010973607317098618));

            c.headingFeedback.set(Controller.proportional(5.006299838728706));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04229571819651968, 0.010950766167994097));

            c.linearBrakeCoefficients.set(Matrix.diag(0.10510081260598154, 0.046043915726641196));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014232079573974425, 0.002433069976482845));

            c.maxAchievableForwardVelocity.set(73.84358979581263);
            c.maxAchievableStrafeVelocity.set(54.72310939860315);
            c.naturalForwardDeceleration.set(30.952686255283027);
            c.naturalStrafeDeceleration.set(73.90994820445482);
        }
    );

    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static PinpointLocalizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    /** Builds the Foresight algorithm from {@link #foresightConfig}. */
    public static Foresight createAlgorithm() {
        return new Foresight(foresightConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap), createAlgorithm());
    }
}
