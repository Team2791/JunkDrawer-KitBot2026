package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Replay implementation of {@link QuestIO} for simulation/log replay.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation - all QuestNav data is
 * read directly from the logged {@link QuestIO.QuestData} values.
 *
 * <p>All methods are no-ops because all replayed data is immutable.
 */
public class QuestReplay extends QuestIO {

    @Override
    protected void reset(Pose3d pose) {}

    @Override
    public void update() {}
}
