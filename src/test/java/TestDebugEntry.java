import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.networktables.DebugEntry;
import frc.robot.subsystems.ExampleSubsystem;
import java.util.HashMap;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestDebugEntry {
  private ExampleSubsystem subsystem;
  private HashMap<String, NetworkTableEntry> entries;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    entries = new HashMap<>();
    subsystem = new ExampleSubsystem();
  }

  @Test
  public void raiseDupEntryError() {
    // Duplicate Entry
    entries.put("test", mock(NetworkTableEntry.class));

    new DebugEntry<Double>(0.0, "test", subsystem);

    // Verify that the warning was reported
    // TODO: Unsure if this actually checks if the DebugEntry reported the warning, or if the DriverStation class is capable of reporting a warning.
    DriverStation.reportWarning(
        "Duplicate ShuffleboardEntry on " + subsystem.getName() + " tab: " + "test", false);
  }
}
