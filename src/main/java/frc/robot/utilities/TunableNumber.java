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

public class TunableNumber extends SubsystemBase implements DoubleSupplier {
  private static NetworkTableInstance inst;
  private static ShuffleboardTab tuningTab;

  private double value;
  private double defaultValue;
  private Consumer<Double> consumer;
  private DoubleTopic doubleTopic;
  private DoubleSubscriber doubleSub;

  public TunableNumber(String name, double defaultValue, Subsystem subsystem) {
    this(name, defaultValue, (ignored) -> {}, subsystem);
  }

  public TunableNumber(
      String name, double defaultValue, Consumer<Double> consumer, Subsystem subsystem) {
    tuningTab = Shuffleboard.getTab(subsystem.getName());
    this.value = defaultValue;
    this.defaultValue = defaultValue;
    this.consumer = consumer;

    if (!Robot.isCompetition) {
      inst = NetworkTableInstance.getDefault();
      tuningTab.add(name, this.defaultValue);

      doubleTopic = inst.getDoubleTopic(name);
      doubleSub =
          doubleTopic.subscribe(
              this.defaultValue, PubSubOption.pollStorage(0), PubSubOption.periodic(1));
    }
  }

  public void periodic() {
    if (!Robot.isCompetition) {
      value = doubleSub.get(this.defaultValue);
      consumer.accept(value);
    }
  }

  public double getAsDouble() {
    return value;
  }
}
