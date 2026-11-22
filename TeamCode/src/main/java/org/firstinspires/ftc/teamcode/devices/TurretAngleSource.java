package org.firstinspires.ftc.teamcode.devices;

/**
 * Where the turret is pointing. TurretSubsystem only talks to this, so the sensor can change without touching the
 * control loop: {@link AxonTurretAngleSource} (Axon feedback wire, bot 1.5 today) now, a REV Through Bore encoder
 * on the turret later.
 *
 * Turret angle convention (degrees): 0 = Pollen pointing straight out the robot's RIGHT side (bot 1.0's fixed shot
 * direction), positive = counterclockwise seen from above (toward the robot's front), same sense as field heading.
 */
public interface TurretAngleSource {

    /** Read the sensor. Call exactly once per loop, before any getter. */
    void update();

    /** False when the sensor is missing from the robot configuration. */
    boolean isConnected();

    /** True once {@link #setHome} has been called (here or in an earlier OpMode, if the source carries it over). */
    boolean isHomed();

    /**
     * Turret angle in degrees (see the convention above), continuous (no wrap: two turns = 720). Before homing it is
     * still continuous but offset by an unknown amount, so it can measure relative motion (homing uses that).
     */
    double getAngleDeg();

    /** Declare that the turret is at {@code turretDeg} right now (called when the Hall sensor trips). */
    void setHome(double turretDeg);

    /** Forget the home, e.g. before re-homing. */
    void clearHome();
}
