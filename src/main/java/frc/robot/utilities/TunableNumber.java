package frc.robot.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class TunableNumber extends SubsystemBase implements DoubleSupplier {
  private static ShuffleboardTab tuningTab;
  private GenericEntry numberEntry;
  private double value;
  private Consumer<Double> consumer;

  public TunableNumber(String name, double defaultValue, Subsystem subsystem) {
    this(name, defaultValue, (ignored) -> {}, subsystem);
  }

  public TunableNumber(
      String name, double defaultValue, Consumer<Double> consumer, Subsystem subsystem) {
    tuningTab = Shuffleboard.getTab(subsystem.getName());
    this.value = defaultValue;
    this.consumer = consumer;
    if (!Robot.isCompetition) {
      numberEntry = tuningTab.add(name, defaultValue).getEntry();
    }
  }

  public void periodic() {
    if (!Robot.isCompetition) {
      consumer.accept(numberEntry.getDouble(value));
      value = numberEntry.getDouble(value);
    }
  }

  public double get() {
    return value;
  }

  public double getAsDouble() {
    return value;
  }
}
