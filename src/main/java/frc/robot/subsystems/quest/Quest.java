package frc.robot.subsystems.quest;

import static frc.robot.constants.VisionConstants.kBotToQuest;
import static frc.robot.constants.VisionConstants.kQuestDevs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.VisionConstants.VisionMeasurement;
import frc.robot.util.Periodic;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.function.Consumer;

/**
 * QuestNav vision subsystem for Meta Quest-based pose estimation.
 *
 * <p>This subsystem provides additional vision-based localization using a Meta Quest device
 * running QuestNav. Like Photon, this does not extend SubsystemBase because it operates
 * independently of the command scheduler.
 *
 * <p>The Quest system:
 * <ul>
 *   <li>Uses inside-out pose tracking from the Quest device
 *   <li>Reports pose measurements to the drivetrain's odometry system
 *   <li>Can improve localization accuracy in GPS-denied environments
 *   <li>Provides independent verification of robot position
 * </ul>
 */
public class Quest {

    /** The QuestNav instance for pose tracking. */
    final QuestNav quest;

    /** Callback to send vision measurements to the drivetrain. */
    final Consumer<VisionMeasurement> addMeasurement;

    /**
     * Constructs a Quest vision system.
     *
     * Initializes the QuestNav instance and registers periodic updates.
     * Vision measurements will be sent to the provided callback function.
     *
     * @param addMeasurement Callback for reporting vision measurements
     */
    public Quest(Consumer<VisionMeasurement> addMeasurement) {
        this.quest = new QuestNav();
        this.addMeasurement = addMeasurement;

        // Register periodic update with command scheduler
        Periodic.schedule(this::update);
    }

    /**
     * Resets the Quest NavBoard's pose estimate.
     *
     * Converts the robot pose to the Quest frame of reference and sets it.
     * Should be called when the robot's true position is known (e.g., at start of match).
     *
     * @param pose The robot's current pose to sync with Quest
     */
    public void reset(Pose2d pose) {
        // Convert 2D pose to 3D
        Pose3d bot = new Pose3d(pose);

        // Transform to Quest frame of reference
        Pose3d quest = bot.transformBy(kBotToQuest);

        // Update Quest's pose estimate
        this.quest.setPose(quest);
    }

    /**
     * Periodic update method called every robot tick.
     *
     * Retrieves new pose frames from the Quest device and converts them to
     * vision measurements for the drivetrain's odometry system.
     */
    void update() {
        // Get all new pose frames from Quest since last update
        PoseFrame[] frames = this.quest.getAllUnreadPoseFrames();
        if (frames.length == 0) return;

        // Process each pose frame
        for (PoseFrame frame : frames) {
            // Skip frames that don't have valid tracking
            if (!frame.isTracking()) continue;

            // Extract pose data from frame
            double ts = frame.dataTimestamp();
            Pose3d quest = frame.questPose3d();

            // Transform from Quest frame to robot frame
            Pose3d bot = quest.transformBy(kBotToQuest.inverse());

            // Create and report vision measurement
            VisionMeasurement est = new VisionMeasurement(bot, kQuestDevs, ts);
            addMeasurement.accept(est);
        }
    }
}
