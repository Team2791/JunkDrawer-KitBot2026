package frc.robot.subsystems.quest;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.alerter.Alerter;
import gg.questnav.questnav.QuestNav;

/**
 * Meta 3S implementation of {@link QuestIO} for Meta Quest-based pose estimation.
 *
 * <p>This class interfaces with the Meta 3S device using the QuestNav library
 * to obtain pose frames for vision-based localization.
 */
public class Meta3S extends QuestIO {

    /** The QuestNav instance for pose tracking. */
    final QuestNav quest;

    /**
     * Constructs a Meta3S vision system.
     *
     * <p>Initializes the QuestNav instance. {@link QuestIO#data} will be updated
     * with the latest readings on each {@link update()} call.
     */
    public Meta3S() {
        this.quest = new QuestNav();

        Alerter.getInstance().register(
            "Meta Quest 3S",
            quest,
            QuestNav::isConnected
        );
    }

    @Override
    public void update() {
        quest.commandPeriodic();

        data.connected = quest.isConnected();
        data.tracking = quest.isTracking();
        data.readings = quest.getAllUnreadPoseFrames();
        data.battery = quest.getBatteryPercent().orElse(-1);
        data.latency = Milliseconds.of(quest.getLatency());
    }

    @Override
    protected void reset(Pose3d pose) {
        quest.setPose(pose);
    }
}
