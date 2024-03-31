// Temporary path & name for the file.
package frc.robot.utilities;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import java.util.HashMap;
import java.util.function.Consumer;

public class DebugEntry<T> {

  private static final DataLog datalog = DataLogManager.getLog();
  private static final HashMap<String, SimpleWidget> entries = new HashMap<String, SimpleWidget>();

  private ShuffleboardTab networkTab;
  private SimpleWidget networkEntry;
  private Integer localEntry;
  private Consumer<T> localConsumer;
  private final String name;
  private T lastValue;

  public DebugEntry(T defaultValue, String name, Subsystem subsystem) {
    this.name = name;
    this.lastValue = defaultValue;
    // if Double
    if (defaultValue instanceof Double) {
      localEntry = datalog.start("/" + subsystem.getName() + "/" + name, "double");
      localConsumer = (a) -> datalog.appendDouble(localEntry, (Double) a, 0);

      // if String
    } else if (defaultValue instanceof String) {
      localEntry = datalog.start("/" + subsystem.getName() + "/" + name, "string");
      localConsumer = (a) -> datalog.appendString(localEntry, (String) a, 0);

      // if Boolean
    } else if (defaultValue instanceof Boolean) {
      localEntry = datalog.start("/" + subsystem.getName() + "/" + name, "boolean");
      localConsumer = (a) -> datalog.appendBoolean(localEntry, (Boolean) a, 0);
      // if Other
    } else {
      DriverStation.reportWarning("Unsupported data type.", false);
    }

    if (!Robot.isCompetition && localEntry != null) {
      networkTab = Shuffleboard.getTab(subsystem.getName());

      if (!entries.containsKey(name)) {
        networkEntry = networkTab.add(name, defaultValue);
        entries.put(name, networkEntry);
      } else {
        String errorMessage =
            "Duplicate ShuffleboardEntry on " + networkTab.getTitle() + " tab: " + name;
        DriverStation.reportError(errorMessage, false);
      }
    }
    if (localEntry != null) localConsumer.accept(defaultValue);
  }

  public DebugEntry<T> withPosition(int colIndex, int rowIndex) {
    if (!Robot.isCompetition) networkEntry.withPosition(colIndex, rowIndex);
    return this;
  }

  public DebugEntry<T> withSize(int width, int height) {
    if (!Robot.isCompetition) networkEntry.withSize(width, height);
    return this;
  }

  public void log(T newValue) {
    try {
      if (!Robot.isCompetition && lastValue != newValue) {
        networkEntry.getEntry().setValue(newValue);
        lastValue = newValue;
      }
      localConsumer.accept(newValue);
    } catch (NullPointerException e) {
      DriverStation.reportError("Invalid type for log " + name, false);
    }
  }
}
