// Temporary path & name for the file.
package frc.robot.utilities;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import java.util.function.Consumer;

public class DebugEntry<T> {
  private static final DataLog datalog = DataLogManager.getLog();

  private final String name;
  private T lastValue;
  private Consumer<T> localConsumer;

  public DebugEntry(T defaultValue, String name, Subsystem subsystem) {
    this(defaultValue, name, subsystem.getName());
  }

  public DebugEntry(T defaultValue, String name, String location) {
    this.name = name;
    this.lastValue = defaultValue;
    final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    final String uri = location + "/" + name;
    Consumer<T> sendToNT;
    Consumer<T> logConsumer;
    Topic t = instance.getTopic(uri);
    if (NetworkTablesJNI.getTopicExists(t.getHandle())) {
      DriverStation.reportError("Debug Entry \"" + uri + "\" already exists", false);
      return;
    }

    // if Double
    if (defaultValue instanceof Double) {
      int localEntry = datalog.start(uri, "double");
      logConsumer = (a) -> datalog.appendDouble(localEntry, (Double) a, 0);

      DoublePublisher publisher = instance.getDoubleTopic(uri).publish();
      sendToNT =
          (data) -> {
            publisher.accept((Double) data);
          };

      // if String
    } else if (defaultValue instanceof String) {
      int localEntry = datalog.start(uri, "string");
      logConsumer = (a) -> datalog.appendString(localEntry, (String) a, 0);

      StringPublisher publisher = instance.getStringTopic(uri).publish();
      sendToNT = (data) -> publisher.accept((String) data);
      // if Boolean
    } else if (defaultValue instanceof Boolean) {
      int localEntry = datalog.start(uri, "boolean");
      logConsumer = (a) -> datalog.appendBoolean(localEntry, (Boolean) a, 0);

      BooleanPublisher publisher = instance.getBooleanTopic(uri).publish();
      sendToNT = (data) -> publisher.accept((Boolean) data);
      // if Other
    } else {
      int localEntry = datalog.start(uri, "string");
      logConsumer = (a) -> datalog.appendString(localEntry, (String) a.toString(), 0);

      StringPublisher publisher = instance.getStringTopic(uri).publish();
      sendToNT = (data) -> publisher.accept(data.toString());
    }

    if (!Robot.isCompetition) {
      localConsumer =
          (a) -> {
            sendToNT.accept(a);
            logConsumer.accept(a);
          };
    } else {
      localConsumer = (a) -> logConsumer.accept(a);
    }

    localConsumer.accept(defaultValue);
  }

  public DebugEntry<T> withPosition(int colIndex, int rowIndex) {
    return this;
  }

  public DebugEntry<T> withSize(int width, int height) {
    return this;
  }

  public void log(T newValue) {
    try {
      if (!Robot.isCompetition && lastValue != newValue) {
        lastValue = newValue;
      }
      localConsumer.accept(newValue);
    } catch (NullPointerException e) {
      DriverStation.reportError("Invalid type for log " + name, false);
    }
  }
}
