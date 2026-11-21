package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PoseStorage.Alliance;

/**
 * Picks our alliance's up-facing HIVE CELL and sets launcher velocity from the distance to it. The bot 1.0 aim (robot
 * turns itself to face the CELL while gp2 X is held) was removed Nov 21 2026; bot 1.5 aims with a turret instead.
 *
 * Field (Pedro inches; Red wall x = 0, audience wall y = 0). From the BIOBUZZ manual (Section 9, TU06): the HIVE
 * Structure is at the field center, the red and blue HIVES are 25.5 in apart center to center (red toward the red
 * wall), and the up CELL's opening faces outward (toward the audience wall or the far wall), centered ~59.6 in above
 * the tiles. The opening's x/y below are worked out from the manual's figures (~+/-2 in).
 *
 * We always target the CELL on the robot's half of the field (audience or far side, from the pose); nothing reads the
 * camera and no button selects it.
 *
 * Robot (bot 1.5, Pollen only): the Pollen launcher rides a turret. getTurretTargetDeg() is the turret angle that points
 * Pollen at the CELL from the turret pivot (turretForwardOffset / turretLeftOffset from the tracking center), in the
 * TurretSubsystem convention: 0 = straight out the robot's right side, + = counterclockwise. The pivot offsets are
 * placeholders (bot 1.0's Pollen position) until the bot 1.5 CAD numbers are in. The shot is assumed to leave along
 * a line through the pivot.
 *
 * Velocity: the base target comes from a table of (distance from the turret pivot to the CELL ->
 * ticks/s), linear between points and flat past the ends (measured Nov 18). Without a known pose the base velocity
 * is NaN and LauncherSubsystem falls back to its fixed default.
 */
@Configurable
public class AutoAimSubsystem extends Subsystem {

    // CELL opening centers (x of each HIVE, y of each CELL side)
    public static double redHiveX = 58.0;
    public static double blueHiveX = 83.5;
    public static double audienceCellY = 55.0;
    public static double farCellY = 86.5;

    // Turret pivot position, inches from the tracking center (+ forward, + left). PLACEHOLDER: set from bot 1.5 CAD.
    public static double turretForwardOffset = -6.148;
    public static double turretLeftOffset = 0.0;

    // Distance (in) -> velocity (ticks/s) table, 5 points, flat past the ends. Edit live in Panels.
    // Measured Nov 18 2026 on bot 1.0 (Red audience CELL); same speed straight or angled. Re-check on bot 1.5.
    public static double dist1 = 33, dist2 = 44, dist3 = 50, dist4 = 55, dist5 = 60;
    public static double pollenVel1 = 1300, pollenVel2 = 1320, pollenVel3 = 1340, pollenVel4 = 1380, pollenVel5 = 1440;

    public enum CellSide { AUDIENCE, FAR }

    private final PoseSource pose;

    /**
     * @param pose field pose + alliance source: {@link LocalizerSubsystem} in TeleOp, {@link FollowerPoseSource} (the
     *             Pedro Follower) in autonomous.
     */
    public AutoAimSubsystem(PoseSource pose) {
        this.pose = pose;
    }

    /**
     * The CELL we aim at: whichever of our two is on the robot's half of the field. The two CELLS sit at audienceCellY
     * and farCellY, so the boundary is halfway between them - a robot nearer the audience wall targets the audience
     * CELL, otherwise the far CELL. Picked from the pose every loop; no driver input selects it.
     */
    public CellSide getTargetCell() {
        return pose.getY() < (audienceCellY + farCellY) / 2 ? CellSide.AUDIENCE : CellSide.FAR;
    }

    public boolean isAvailable() { return pose.isPoseKnown(); }

    public double getTargetX() { return pose.getAlliance() == Alliance.BLUE ? blueHiveX : redHiveX; }
    public double getTargetY() { return getTargetCell() == CellSide.AUDIENCE ? audienceCellY : farCellY; }

    /** Turret pivot x on the field (in). */
    private double pivotX() {
        double h = pose.getHeading();
        return pose.getX() + turretForwardOffset * Math.cos(h) - turretLeftOffset * Math.sin(h);
    }

    /** Turret pivot y on the field (in). */
    private double pivotY() {
        double h = pose.getHeading();
        return pose.getY() + turretForwardOffset * Math.sin(h) + turretLeftOffset * Math.cos(h);
    }

    /** Distance (in) from the turret pivot to the CELL, at the robot's current pose. */
    public double getPollenDistance() {
        return Math.hypot(getTargetX() - pivotX(), getTargetY() - pivotY());
    }

    /**
     * Turret angle (deg, -180..180) that points Pollen at the CELL; NaN when the pose is not known. The turret's 0
     * direction on the field is heading - 90 deg (the robot's right side), so turret = bearing - heading + 90.
     * TurretSubsystem clamps it to its travel; check {@link #isTurretTargetInRange} to know whether it can get there.
     */
    public double getTurretTargetDeg() {
        if (!isAvailable()) return Double.NaN;
        double bearing = Math.atan2(getTargetY() - pivotY(), getTargetX() - pivotX());
        double deg = Math.toDegrees(bearing - pose.getHeading()) + 90.0;
        deg %= 360.0;
        if (deg > 180) deg -= 360;
        if (deg <= -180) deg += 360;
        return deg;
    }

    /** True when the CELL is within the turret's travel (TurretSubsystem.minAngleDeg .. maxAngleDeg). */
    public boolean isTurretTargetInRange() {
        double t = getTurretTargetDeg();
        return !Double.isNaN(t) && t >= TurretSubsystem.minAngleDeg && t <= TurretSubsystem.maxAngleDeg;
    }

    /** Base Pollen velocity from the table; NaN when the pose is not known (the launcher then uses its default). */
    public double getPollenBaseVelocity() {
        if (!isAvailable()) return Double.NaN;
        return interpolate(getPollenDistance(), pollenVel1, pollenVel2, pollenVel3, pollenVel4, pollenVel5);
    }

    private static double interpolate(double d, double v1, double v2, double v3, double v4, double v5) {
        double[] ds = {dist1, dist2, dist3, dist4, dist5};
        double[] vs = {v1, v2, v3, v4, v5};
        if (d <= ds[0]) return vs[0];
        for (int i = 1; i < ds.length; i++) {
            if (d <= ds[i]) {
                double span = ds[i] - ds[i - 1];
                double t = span > 0 ? (d - ds[i - 1]) / span : 1;
                return vs[i - 1] + t * (vs[i] - vs[i - 1]);
            }
        }
        return vs[vs.length - 1];
    }

    @Override
    public void init() { }

    @Override
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    public void update() { }   // nothing to do each loop: the getters compute from the current pose

    private String cellName() {
        return (pose.getAlliance() == Alliance.BLUE ? "BLUE " : "RED ") + getTargetCell() + " CELL";
    }

    /** Target and distance-based speed lines for the Driver Station. */
    public void addTelemetry(Telemetry telemetry) {
        if (!isAvailable()) {
            telemetry.addLine("TARGET: unavailable (alliance / pose not set) - fixed launcher speeds");
            return;
        }
        telemetry.addLine("TARGET: " + cellName() + "  (auto: the CELL on your half of the field)");
        telemetry.addData("Pollen dist / base vel", "%.1f in / %.0f", getPollenDistance(), getPollenBaseVelocity());
        telemetry.addData("Turret aim", "%.1f deg%s", getTurretTargetDeg(),
                isTurretTargetInRange() ? "" : "  OUT OF TURRET RANGE - turn the robot");
    }

    @Override
    public void stop() { }
}
