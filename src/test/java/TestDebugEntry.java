import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyBoolean;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.networktables.DebugEntry;
import frc.robot.subsystems.ExampleSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

public class TestDebugEntry {
  private ExampleSubsystem subsystem;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    subsystem = new ExampleSubsystem();
  }

  @Test
  public void raiseDupEntryError() {
    // Duplicate Entry
    try (MockedStatic<DriverStation> mockedFactory = Mockito.mockStatic(DriverStation.class)) {
      mockedFactory
          .when(() -> DriverStation.reportWarning(anyString(), anyBoolean()))
          .thenCallRealMethod();
      new DebugEntry<Double>(0.0, "test", subsystem);
      new DebugEntry<Double>(0.0, "test", subsystem);
      mockedFactory.verify(() -> DriverStation.reportWarning(anyString(), anyBoolean()), times(1));
    }
  }

  @Test
  public void raiseDataTypeError() {
    // Unsupported data type
    try (MockedStatic<DriverStation> mockedFactory = Mockito.mockStatic(DriverStation.class)) {
      mockedFactory
          .when(() -> DriverStation.reportWarning(anyString(), anyBoolean()))
          .thenCallRealMethod();
      try (MockedConstruction<DebugEntry> mockedDebugEntry =
          Mockito.mockConstruction(
              DebugEntry.class,
              (mock, context) -> {
                DriverStation.reportWarning(anyString(), anyBoolean());
              })) {
        new DebugEntry<String>("nonDouble", "test", subsystem);
      }
      mockedFactory.verify(() -> DriverStation.reportWarning(anyString(), anyBoolean()), times(1));
    }
  }

  @Test
  public void logInvalidTypeError() {
    // Invalid data type for log
    try (MockedStatic<DriverStation> mockedFactory = Mockito.mockStatic(DriverStation.class)) {
      mockedFactory
          .when(() -> DriverStation.reportError(anyString(), anyBoolean()))
          .thenCallRealMethod();
      try (MockedConstruction<DebugEntry> mockedDebugEntry =
          Mockito.mockConstruction(
              DebugEntry.class,
              (mock, context) -> {
                try {

                  mock.log(any());
                  DriverStation.reportError(anyString(), anyBoolean());
                } catch (Exception e) {
                  fail("Unexpected exception: " + e.getMessage());
                }
              })) {
        new DebugEntry<String>("nonDouble", "test", subsystem);
      }
      mockedFactory.verify(() -> DriverStation.reportError(anyString(), anyBoolean()), times(1));
    }
  }
}
