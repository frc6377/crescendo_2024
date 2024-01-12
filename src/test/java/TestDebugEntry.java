import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.networktables.DebugEntry;
import frc.robot.subsystems.ExampleSubsystem;
import java.util.HashMap;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

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

    try (MockedStatic<DriverStation> mockedFactory = Mockito.mockStatic(DriverStation.class)) {
      mockedFactory
          .when(() -> DriverStation.reportWarning(anyString(), anyBoolean()))
          .thenCallRealMethod();
      new DebugEntry<Double>(0.0, "test", subsystem);
      new DebugEntry<Double>(0.0, "test", subsystem);
      mockedFactory.verify(() -> DriverStation.reportWarning(anyString(), anyBoolean()), times(1));
    }

  }
}
