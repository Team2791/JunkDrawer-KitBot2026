package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.QuestNav;

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
    }

    @Override
    public void update() {
        data.connected = quest.isConnected();
        data.readings = quest.getAllUnreadPoseFrames();
    }

    @Override
    protected void reset(Pose3d pose) {
        quest.setPose(pose);
    }
}
