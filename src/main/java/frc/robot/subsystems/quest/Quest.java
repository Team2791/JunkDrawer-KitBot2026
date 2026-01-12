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

public class Quest {

    final QuestNav quest;
    final Consumer<VisionMeasurement> addMeasurement;

    public Quest(Consumer<VisionMeasurement> addMeasurement) {
        this.quest = new QuestNav();
        this.addMeasurement = addMeasurement;

        Periodic.schedule(this::update);
    }

    public void reset(Pose2d pose) {
        Pose3d bot = new Pose3d(pose);
        Pose3d quest = bot.transformBy(kBotToQuest);
        this.quest.setPose(quest);
    }

    void update() {
        PoseFrame[] frames = this.quest.getAllUnreadPoseFrames();
        if (frames.length == 0) return;

        for (PoseFrame frame : frames) {
            if (!frame.isTracking()) continue;

            double ts = frame.dataTimestamp();
            Pose3d quest = frame.questPose3d();
            Pose3d bot = quest.transformBy(kBotToQuest.inverse());

            VisionMeasurement est = new VisionMeasurement(bot, kQuestDevs, ts);
            addMeasurement.accept(est);
        }
    }
}
