package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Bot 1.5 Hive vision: which side of OUR Hive is up, whether it has settled, and how sure we are. Camera only - no
 * ball counting (both robots on our alliance, and sometimes opponents, shoot into it).
 *
 * The webcam is on the front of the turret, tilted 45 deg up. Autonomous points the turret at where a cell would be
 * when raised and tells this subsystem with {@link #setLookingAtHive}. Only the raised cell's tags can be seen, so:
 *  - tags of a cell visible (one tag of a partly blocked cluster is enough) = that side is up;
 *  - aimed at a cell and none of its tags for absentSec = that side is down, so the other side is up.
 * The decisions themselves live in {@link HiveTracker} (plain Java, tested off the robot).
 *
 * This subsystem only reports. Autonomous owns poses, paths, the turret and the clock:
 *   getTargetSide() -> where to shoot, isReadyToShoot() -> fire, getSecUntilSettled() -> how long to wait.
 *
 * Camera modes (Autonomous sets them; see the plan doc):
 *   OFF  - volleys (flywheel recovery needs the fastest loop); AprilTag processing disabled, camera keeps streaming
 *   SLOW - driving and pickup: one processed frame every slowIntervalSec
 *   FAST - INIT, waiting for the first tip, and at the shooting spot when not settled yet
 * Switching uses setProcessorEnabled (instant). Live view and tag drawing are off to save CPU.
 *
 * Tag IDs (SDK 12.0 game database): RED SCORING 30-33, RED AUDIENCE 34-37, BLUE AUDIENCE 38-41, BLUE SCORING 42-45.
 */
@Configurable
public class HiveVisionSubsystem extends Subsystem {

    public enum Alliance { RED, BLUE }

    public enum HiveSide {
        RED_AUDIENCE, RED_SCORING, BLUE_AUDIENCE, BLUE_SCORING;
        public HiveTracker.Cell cell() {
            return (this == RED_AUDIENCE || this == BLUE_AUDIENCE) ? HiveTracker.Cell.AUDIENCE : HiveTracker.Cell.SCORING;
        }
        public Alliance alliance() { return (this == RED_AUDIENCE || this == RED_SCORING) ? Alliance.RED : Alliance.BLUE; }
    }

    public enum Mode { OFF, SLOW, FAST }

    // Camera
    public static String webcamName = "Webcam 1";
    public static int cameraWidth = 640, cameraHeight = 480;   // read once, when the subsystem is built
    public static double decimation = 2;                       // 1 = sees farther, slower; 3 = faster
    public static double exposureMs = 0;                       // 0 = auto; ~5-8 ms with gain cuts motion blur
    public static int gain = 200;                              // used only with a manual exposure
    // Sampling
    public static double slowIntervalSec = 0.5;
    // Decisions (copied into HiveTracker every loop; see HiveTracker for what each does)
    public static double rateWindowSec = 0.4;
    public static double settleRateDegPerSec = 5.0;
    public static double settleSec = 0.25;
    public static double absentSec = 0.75;
    public static int absentFrames = 2;
    public static double tiltTimeSec = 3.5;
    public static double staleSec = 1.5;
    public static int confirmSamples = 2;
    // Telemetry is skipped in matches unless this is on (it costs loop time)
    public static boolean debugTelemetry = false;

    private final AprilTagProcessor aprilTag;
    private final VisionPortal portal;
    private final HiveTracker tracker = new HiveTracker();
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime slowTimer = new ElapsedTime();

    private Alliance alliance;
    private Mode mode = Mode.FAST;
    private boolean looking = false;
    private HiveSide lookSide = null;
    private boolean exposureApplied = false;

    // For telemetry / field measurements
    private final List<HiveTracker.Sighting> lastSightings = new ArrayList<>();
    private double lastFrameT = Double.NaN;
    private double lastLatencyMs = Double.NaN;
    private int framesProcessed = 0;

    public HiveVisionSubsystem(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        WebcamName cam = hardwareMap.tryGet(WebcamName.class, webcamName);
        if (cam == null) {
            aprilTag = null;
            portal = null;
            return;
        }
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setDrawTagID(false)
                .build();
        aprilTag.setDecimation((float) decimation);
        portal = new VisionPortal.Builder()
                .setCamera(cam)
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .enableLiveView(false)
                .addProcessor(aprilTag)
                .build();
    }

    // ---- commands from Autonomous / TeleOp ----

    /** Which alliance's Hive to watch. Clears what was learned (e.g. when the alliance is picked during INIT). */
    public void setAlliance(Alliance a) {
        if (a != alliance) {
            alliance = a;
            tracker.reset();
        }
    }

    public void setMode(Mode m) { mode = m; }

    /**
     * Tell the tracker where the camera is aimed. {@code on} = the turret is on target at that cell's raised position
     * and the robot is at a spot where that cell's tags are visible when it is up (checked on the field). Only then
     * does "no tags" count as "that side is down".
     */
    public void setLookingAtHive(boolean on, HiveSide side) {
        looking = on && side != null && side.alliance() == alliance;
        lookSide = looking ? side : null;
    }

    public void resetTracker() { tracker.reset(); }

    // ---- answers ----

    public boolean isConnected() { return portal != null; }

    /** The side that is up, or coming up while tilting. Null until the camera has seen or ruled out a side. */
    public HiveSide getTargetSide() { return toSide(tracker.getTarget()); }
    public HiveTracker.State getState() { return tracker.getState(); }
    public HiveTracker.Evidence getEvidence() { return tracker.getEvidence(); }
    public boolean isReadyToShoot() { return tracker.isReadyToShoot(clock.seconds()); }
    /** 0 when settled, a countdown while tilting, NaN when unknown. */
    public double getSecUntilSettled() { return tracker.getSecUntilSettled(clock.seconds()); }
    public boolean isConfident() { return tracker.isConfident(clock.seconds()); }
    public double getSecSinceSeen() { return tracker.getSecSinceSeen(clock.seconds()); }
    /** True when aimed at a cell and it has shown no tags for at least {@code sec} (and enough frames). */
    public boolean noTagsAtSpot(double sec) {
        return looking && tracker.getNoTagsFrames() >= absentFrames && tracker.getNoTagsSec(clock.seconds()) >= sec;
    }
    /** Goes up every time the side changes, so Auto can tell it flipped since it last looked. */
    public int getSideChangeCount() { return tracker.getSideChanges(); }
    public Mode getMode() { return mode; }
    public Alliance getAlliance() { return alliance; }

    private HiveSide toSide(HiveTracker.Cell c) {
        if (c == null) return null;
        if (alliance == Alliance.RED) return c == HiveTracker.Cell.AUDIENCE ? HiveSide.RED_AUDIENCE : HiveSide.RED_SCORING;
        return c == HiveTracker.Cell.AUDIENCE ? HiveSide.BLUE_AUDIENCE : HiveSide.BLUE_SCORING;
    }

    // ---- Subsystem ----

    @Override public void init() { clock.reset(); slowTimer.reset(); }

    /** Runs during INIT too, so the start side is known before START. */
    @Override public void init_loop() { update(); }

    @Override public void start() { }

    @Override
    public void update() {
        double t = clock.seconds();
        copyTunables();
        if (portal == null) return;
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        applyExposureOnce();

        boolean enabled = portal.getProcessorEnabled(aprilTag);
        switch (mode) {
            case OFF:
                if (enabled) portal.setProcessorEnabled(aprilTag, false);
                tracker.predict(t);
                return;
            case FAST:
                if (!enabled) portal.setProcessorEnabled(aprilTag, true);
                break;
            case SLOW:
                if (!enabled && slowTimer.seconds() >= slowIntervalSec) {
                    portal.setProcessorEnabled(aprilTag, true);
                    aprilTag.getFreshDetections();   // drop any result from before the pause
                }
                break;
        }

        List<AprilTagDetection> fresh = aprilTag.getFreshDetections();   // null = no new frame processed
        if (fresh == null) {
            tracker.predict(t);
            return;
        }
        framesProcessed++;
        lastFrameT = t;
        lastSightings.clear();
        for (AprilTagDetection d : fresh) {
            HiveTracker.Sighting s = toSighting(d);
            if (s != null) lastSightings.add(s);
            lastLatencyMs = (System.nanoTime() - d.frameAcquisitionNanoTime) / 1e6;
        }
        tracker.addFrame(t, lastSightings, looking, looking ? lookSide.cell() : null);

        if (mode == Mode.SLOW) {   // one frame per interval, then pause the processing again
            portal.setProcessorEnabled(aprilTag, false);
            slowTimer.reset();
        }
    }

    @Override
    public void stop() {
        if (portal != null) portal.close();
    }

    /** Our alliance's tag or cluster -> a sighting; anything else (other alliance, other field tags) -> null. */
    private HiveTracker.Sighting toSighting(AprilTagDetection d) {
        double pitch = d.ftcPose != null ? d.ftcPose.pitch : Double.NaN;
        if (d instanceof AprilTagSingleDetection) {
            int id = ((AprilTagSingleDetection) d).id;
            HiveTracker.Cell cell = cellOfTag(id);
            return cell == null ? null : new HiveTracker.Sighting(cell, "id" + id, 1, pitch);
        }
        if (d instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            String name = c.metadata != null ? c.metadata.name : null;
            HiveTracker.Cell cell = cellOfCluster(name);
            if (cell == null) return null;
            int tags = (int) Math.round(4 * c.percentClusterFound / 100.0);   // part of a cluster still counts
            return new HiveTracker.Sighting(cell, name, Math.max(1, tags), pitch);
        }
        return null;
    }

    private HiveTracker.Cell cellOfTag(int id) {
        if (alliance == Alliance.RED) {
            if (id >= 30 && id <= 33) return HiveTracker.Cell.SCORING;
            if (id >= 34 && id <= 37) return HiveTracker.Cell.AUDIENCE;
        } else {
            if (id >= 38 && id <= 41) return HiveTracker.Cell.AUDIENCE;
            if (id >= 42 && id <= 45) return HiveTracker.Cell.SCORING;
        }
        return null;
    }

    private HiveTracker.Cell cellOfCluster(String name) {
        if (name == null) return null;
        String color = alliance == Alliance.RED ? "RED" : "BLUE";
        if (name.equals(color + " AUDIENCE")) return HiveTracker.Cell.AUDIENCE;
        if (name.equals(color + " SCORING")) return HiveTracker.Cell.SCORING;
        return null;
    }

    private void copyTunables() {
        tracker.rateWindowSec = rateWindowSec;
        tracker.settleRateDegPerSec = settleRateDegPerSec;
        tracker.settleSec = settleSec;
        tracker.absentSec = absentSec;
        tracker.absentFrames = absentFrames;
        tracker.tiltTimeSec = tiltTimeSec;
        tracker.staleSec = staleSec;
        tracker.confirmSamples = confirmSamples;
        if (aprilTag != null) aprilTag.setDecimation((float) decimation);
    }

    private void applyExposureOnce() {
        if (exposureApplied || exposureMs <= 0) return;
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);
            portal.getCameraControl(GainControl.class).setGain(gain);
        } catch (RuntimeException e) {
            // Camera without these controls: keep auto exposure.
        }
        exposureApplied = true;
    }

    // ---- telemetry ----

    public void addTelemetry(Telemetry telemetry) { addTelemetry(telemetry, debugTelemetry); }

    public void addTelemetry(Telemetry telemetry, boolean show) {
        if (!show) return;
        double t = clock.seconds();
        telemetry.addLine("--- Hive vision ---");
        if (portal == null) {
            telemetry.addData("Camera", "NOT CONFIGURED (%s)", webcamName);
            return;
        }
        telemetry.addData("Camera / mode", "%s / %s   fps %.0f", portal.getCameraState(), mode, portal.getFps());
        telemetry.addData("Target side", "%s  (%s, %s)", getTargetSide(), tracker.getEvidence(),
                isConfident() ? "confident" : "NOT confident");
        telemetry.addData("State", "%s%s   ready to shoot: %s", tracker.getState(),
                tracker.isTippingAway() ? " (tipping away)" : "", isReadyToShoot() ? "YES" : "no");
        telemetry.addData("Until settled", "%.1f s", getSecUntilSettled());
        telemetry.addData("Seen", "%s, %d tags, %.1f s ago, angle rate %.1f deg/s", tracker.getSeenCell(),
                tracker.getLastTags(), getSecSinceSeen(), tracker.getLastRateDegPerSec());
        telemetry.addData("Looking at", looking ? lookSide + String.format("   no tags %.1f s", tracker.getNoTagsSec(t)) : "-");
        telemetry.addData("Side changes", tracker.getSideChanges());
        StringBuilder sb = new StringBuilder();
        for (HiveTracker.Sighting s : lastSightings) {
            sb.append(s.source).append(String.format("(%d, %.1f) ", s.tags, s.pitchDeg));
        }
        telemetry.addData("Last frame", "%s   %.1f s ago, latency %.0f ms, frames %d",
                sb.length() > 0 ? sb.toString() : "no Hive tags", t - lastFrameT, lastLatencyMs, framesProcessed);
    }
}
