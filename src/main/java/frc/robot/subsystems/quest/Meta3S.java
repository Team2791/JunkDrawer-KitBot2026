package frc.robot.subsystems.quest;

import gg.questnav.questnav.QuestNav;

public class Meta3S implements QuestIO {

  private final QuestNav quest;

  public Meta3S() {
    this.quest = new QuestNav();
  }

  @Override
  public void updateInputs(QuestIOInputs inputs) {
    inputs.connected = quest.isConnected();
    inputs.readings = quest.getAllUnreadPoseFrames();
  }
}
