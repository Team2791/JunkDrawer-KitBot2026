package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.AutoLog;

public abstract class QuestIO {

    @AutoLog
    public static class QuestData {

        public boolean connected = false;
        public PoseFrame[] readings = new PoseFrame[0];
    }

    /** The current QuestNav data, since the last update() call */
    public final QuestDataAutoLogged data = new QuestDataAutoLogged();

    /** Updates this.data with the current QuestNav data. */
    public abstract void update();

    /**
     * Reset the <b>quest</b> to the specified pose.
     *
     * @param pose The reset pose of the <b>quest</b>
     */
    protected abstract void reset(Pose3d pose);
}
