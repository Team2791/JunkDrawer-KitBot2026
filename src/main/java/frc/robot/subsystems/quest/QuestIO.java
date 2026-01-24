package frc.robot.subsystems.quest;

import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.AutoLog;

public interface QuestIO {
  @AutoLog
  public static class QuestIOInputs {
    public boolean connected = false;
    public PoseFrame[] readings = new PoseFrame[0];
  }

  public default void updateInputs(QuestIOInputs inputs) {}
}
