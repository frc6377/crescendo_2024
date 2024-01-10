// Temporary path & name for the file.
package frc.robot.networktables;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.function.Consumer;

public class DeltaDebugBoard<T> {
  private static Hashtable<String, String> stringTable = new Hashtable<>();
  private static Hashtable<String, Double> doubleTable = new Hashtable<>();
  private static Hashtable<String, Boolean> booleanTable = new Hashtable<>();

  private static final DataLog datalog = DataLogManager.getLog();
  private static final HashMap<String, GenericEntry> entries = new HashMap<String, GenericEntry>();

  private ShuffleboardTab networkTab;
  private GenericEntry networkEntry;
  private Integer localEntry;
  private Consumer<T> localConsumer;
  private final String name;

  public static void putNumber(String key, Double val) {
    if (!doubleTable.containsKey(key) || !doubleTable.get(key).equals(val)) {
      SmartDashboard.putNumber(key, val);
      doubleTable.put(key, val);
    }
  }

  public static void putString(String key, String val) {
    if (!stringTable.containsKey(key) || !stringTable.get(key).equals(val)) {
      SmartDashboard.putString(key, val);
      stringTable.put(key, val);
    }
  }

  public static void putBoolean(String key, boolean val) {
    if (!stringTable.containsKey(key) || !stringTable.get(key).equals(val)) {
      SmartDashboard.putBoolean(key, val);
      booleanTable.put(key, val);
    }
  }

  public DeltaDebugBoard(T defaultValue, String name, SubsystemBase subsystem) {
    this.name = name;
    if (defaultValue instanceof Double) {
      localEntry = datalog.start("/" + subsystem.getName() + "/" + name, "double");
      localConsumer = (a) -> datalog.appendDouble(localEntry, (Double) a, 0);
    }
    if (defaultValue instanceof String) {
      localEntry = datalog.start("/" + subsystem.getName() + "/" + name, "string");
      localConsumer = (a) -> datalog.appendString(localEntry, (String) a, 0);
    }
    if (defaultValue instanceof Boolean) {
      localEntry = datalog.start("/" + subsystem.getName() + "/" + name, "boolean");
      localConsumer = (a) -> datalog.appendBoolean(localEntry, (Boolean) a, 0);
    }
    if (localEntry != null) {
      if (!Robot.isCompetition) {
        networkTab = Shuffleboard.getTab(subsystem.getName());

        if (!entries.containsKey(name)) {
          networkEntry = networkTab.add(name, defaultValue).getEntry();
          entries.put(name, networkEntry);
        } else {
          networkEntry = entries.get(name);
          DriverStation.reportWarning(
              "Duplicate ShuffleboardEntry on " + networkTab.getTitle() + " tab: " + name, false);
        }
      }
      localConsumer.accept(defaultValue);
    }
  }

  public void log(T newValue) {
    try {
      if (!Robot.isCompetition) {
        networkEntry.setValue(newValue);
      }
      localConsumer.accept(newValue);
    } catch (NullPointerException e) {
      DriverStation.reportError("Invalid type for log " + name, false);
    }
  }
}
