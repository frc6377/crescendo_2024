package frc.robot.utilities;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class TunableNumber extends SubsystemBase implements DoubleSupplier {
  private NetworkTableInstance inst;
  private static ShuffleboardTab tuningTab;
  private GenericEntry numberEntry;
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
    this.consumer = consumer;
    if (!Robot.isCompetition) {
      inst = NetworkTableInstance.getDefault();
      numberEntry = tuningTab.add(name, defaultValue).getEntry();
      doubleTopic = inst.getDoubleTopic(name);
      doubleSub = doubleTopic.subscribe(defaultValue);
    }
  }

  public void periodic() {
    if (!Robot.isCompetition) {
      consumer.accept(doubleSub.get(defaultValue));
      value = doubleSub.get(defaultValue);
    }
  }

  public double getAsDouble() {
    return value;
  }
}
