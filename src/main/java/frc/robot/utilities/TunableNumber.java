package frc.robot.utilities;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import javax.annotation.Nullable;

public class TunableNumber extends SubsystemBase implements DoubleSupplier {
  @Nullable private static NetworkTableInstance inst;
  @Nullable private ShuffleboardTab tuningTab;

  private double value;
  private double defaultValue;
  private Consumer<Double> consumer;
  @Nullable private DoubleTopic doubleTopic;
  @Nullable private DoubleSubscriber doubleSub;

  public TunableNumber(String name, double defaultValue, @Nullable Subsystem subsystem) {
    this(name, defaultValue, (ignored) -> {}, subsystem);
  }

  public TunableNumber(
      String name, double defaultValue, Consumer<Double> consumer, @Nullable Subsystem subsystem) {
    if (subsystem != null) {
      tuningTab = Shuffleboard.getTab(subsystem.getName());
    }
    this.value = defaultValue;

    this.defaultValue = defaultValue;
    this.consumer = consumer;

    if (!Robot.isCompetition && tuningTab != null) {
      inst = NetworkTableInstance.getDefault();
      tuningTab.add(name, this.defaultValue);

      doubleTopic = inst.getDoubleTopic(name);
      doubleSub =
          doubleTopic.subscribe(
              this.defaultValue, PubSubOption.pollStorage(2), PubSubOption.periodic(1));
    }
  }

  public void periodic() {
    if (doubleSub != null) {
      value = doubleSub.get(this.defaultValue);
    }
    consumer.accept(value);
  }

  public double getAsDouble() {
    return value;
  }
}
