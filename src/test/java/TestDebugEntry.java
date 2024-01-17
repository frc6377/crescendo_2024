import static org.mockito.ArgumentMatchers.anyBoolean;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.networktables.DebugEntry;
import frc.robot.subsystems.ExampleSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

public class TestDebugEntry {
  private ExampleSubsystem subsystem;

  private class TestSubsystem extends SubsystemBase {}

  private TestSubsystem subsystem2;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    subsystem = new ExampleSubsystem();
    subsystem2 = new TestSubsystem();
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
    try (MockedStatic<DriverStation> mockedFactory = Mockito.mockStatic(DriverStation.class)) {
      mockedFactory
          .when(() -> DriverStation.reportWarning(anyString(), anyBoolean()))
          .thenCallRealMethod();

      DebugEntry<Float> dut =
          new DebugEntry<Float>(Float.valueOf((float) 10.0), "test2", subsystem);
      mockedFactory.verify(() -> DriverStation.reportWarning(anyString(), anyBoolean()), times(1));

      dut.log((float) 10.0);
      mockedFactory.verify(() -> DriverStation.reportWarning(anyString(), anyBoolean()), times(1));
    }
  }

  @Disabled
  @Test
  public void testSameNameDifferentSubsystem() {
    try (MockedStatic<DriverStation> mockedFactory = Mockito.mockStatic(DriverStation.class)) {
      mockedFactory
          .when(() -> DriverStation.reportWarning(anyString(), anyBoolean()))
          .thenCallRealMethod();

      new DebugEntry<Double>(0.0, "test3", subsystem);
      new DebugEntry<Double>(0.0, "test3", subsystem2);
      mockedFactory.verify(() -> DriverStation.reportWarning(anyString(), anyBoolean()), times(0));
    }
  }
}
