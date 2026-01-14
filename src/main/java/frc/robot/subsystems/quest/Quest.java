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
import org.littletonrobotics.junction.Logger;

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

  private final QuestIO io;
  private final QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();
  private final TriConsumer<Pose2d, Double, Matrix<N3, N1>> addMeasurement;

  public Quest(QuestIO io, TriConsumer<Pose2d, Double, Matrix<N3, N1>> addMeasurement) {
    this.io = io;
    this.addMeasurement = addMeasurement;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Quest", inputs);

    // Process each pose frame
    for (PoseFrame frame : inputs.readings) {
      if (!frame.isTracking()) continue;

      double ts = frame.dataTimestamp();
      Pose3d questPose = frame.questPose3d();
      Pose3d botPose = questPose.transformBy(kBotToQuest.inverse());

      addMeasurement.accept(botPose.toPose2d(), ts, kQuestDevs);
    }
  }
}
