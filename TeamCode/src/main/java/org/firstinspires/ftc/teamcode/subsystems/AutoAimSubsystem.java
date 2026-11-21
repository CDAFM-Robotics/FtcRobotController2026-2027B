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
 * Robot: both launchers fire out the robot's RIGHT side (shot direction = heading - 90 deg). Along the robot's length
 * (from CAD, 18 in robot, tracking center 9 in from the front): Pollen launcher 6.148 in behind the center, Nectar
 * 0.623 in ahead.
 *
 * Velocity: each launcher's base target comes from its own table of (distance from that launcher to the CELL ->
 * ticks/s), linear between points and flat past the ends (measured Nov 18). Without a known pose the base velocities
 * are NaN and LauncherSubsystem falls back to its own fixed defaults.
 */
@Configurable
public class AutoAimSubsystem extends Subsystem {

    // CELL opening centers (x of each HIVE, y of each CELL side)
    public static double redHiveX = 58.0;
    public static double blueHiveX = 83.5;
    public static double audienceCellY = 55.0;
    public static double farCellY = 86.5;

    // Launcher positions along the robot's forward axis, inches from the tracking center (+ = forward)
    public static double pollenForwardOffset = -6.148;
    public static double nectarForwardOffset = 0.623;

    // Distance (in) -> velocity (ticks/s) tables, 5 points each, flat past the ends. Edit live in Panels.
    // Measured Nov 18 2026 (Red audience CELL). ~60 in is about the farthest shot (field corner), ~43 in the farthest
    // straight-on shot. Pollen: same speed straight or angled. Nectar: measured on ANGLED (~50 deg) shots.
    public static double dist1 = 33, dist2 = 44, dist3 = 50, dist4 = 55, dist5 = 60;
    public static double pollenVel1 = 1300, pollenVel2 = 1320, pollenVel3 = 1340, pollenVel4 = 1380, pollenVel5 = 1440;
    public static double nectarVel1 = 1480, nectarVel2 = 1540, nectarVel3 = 1540, nectarVel4 = 1540, nectarVel5 = 1560;

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

    /** Distance (in) from the launcher at the given forward offset to the CELL, at the robot's current pose. */
    private double launcherDistance(double forwardOffset) {
        double h = pose.getHeading();
        double lx = pose.getX() + forwardOffset * Math.cos(h);
        double ly = pose.getY() + forwardOffset * Math.sin(h);
        return Math.hypot(getTargetX() - lx, getTargetY() - ly);
    }

    public double getPollenDistance() { return launcherDistance(pollenForwardOffset); }
    public double getNectarDistance() { return launcherDistance(nectarForwardOffset); }

    /** Base Pollen velocity from the table; NaN when the pose is not known (the launcher then uses its default). */
    public double getPollenBaseVelocity() {
        if (!isAvailable()) return Double.NaN;
        return interpolate(getPollenDistance(), pollenVel1, pollenVel2, pollenVel3, pollenVel4, pollenVel5);
    }

    public double getNectarBaseVelocity() {
        if (!isAvailable()) return Double.NaN;
        return interpolate(getNectarDistance(), nectarVel1, nectarVel2, nectarVel3, nectarVel4, nectarVel5);
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
        telemetry.addData("Nectar dist / base vel", "%.1f in / %.0f", getNectarDistance(), getNectarBaseVelocity());
    }

    @Override
    public void stop() { }
}
