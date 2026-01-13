package frc.robot.subsystems.quest;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {

  @FunctionalInterface
  public interface TriConsumer<T, U, V> {
    void accept(T t, U u, V v);
  }

  private static final Transform3d kBotToQuest = new Transform3d(); // TODO: measure and set
  private static final Matrix<N3, N1> kQuestDevs =
      VecBuilder.fill(
          Centimeters.of(2).in(Meters),
          Centimeters.of(2).in(Meters),
          Degrees.of(0.035).in(Radians));

  final QuestNav quest;
  final TriConsumer<Pose2d, Double, Matrix<N3, N1>> addMeasurement;

  public Quest(TriConsumer<Pose2d, Double, Matrix<N3, N1>> addMeasurement) {
    this.quest = new QuestNav();
    this.addMeasurement = addMeasurement;
  }

  public static Matrix<N3, N1> getStdDevs() {
    return kQuestDevs;
  }

  public void reset(Pose2d pose) {
    Pose3d bot = new Pose3d(pose);
    Pose3d quest = bot.transformBy(kBotToQuest);
    this.quest.setPose(quest);
  }

  @Override
  public void periodic() {
    PoseFrame[] frames = this.quest.getAllUnreadPoseFrames();
    if (frames.length == 0) return;

    for (PoseFrame frame : frames) {
      if (!frame.isTracking()) continue;

      double ts = frame.dataTimestamp();
      Pose3d quest = frame.questPose3d();
      Pose3d bot = quest.transformBy(kBotToQuest.inverse());

      addMeasurement.accept(bot.toPose2d(), ts, kQuestDevs);
    }
  }
}
