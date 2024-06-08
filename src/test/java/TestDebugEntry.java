import static org.mockito.ArgumentMatchers.anyBoolean;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.DebugEntry;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

public class TestDebugEntry {
  private TestSubsystem subsystem;

  private class TestSubsystem extends SubsystemBase {}

  private TestSubsystem subsystem2;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    subsystem = new TestSubsystem();
    subsystem2 = new TestSubsystem();
  }

  @AfterEach
  public void cleanupTests() {
    HAL.shutdown();
    subsystem = new TestSubsystem();
    subsystem2 = new TestSubsystem();
  }

  @Disabled
  @Test()
  public void raiseDupEntryError() {
    // Duplicate Entry
    if (Robot.isCompetition) {
      return;
    }
    try (MockedStatic<DriverStation> mockedFactory = Mockito.mockStatic(DriverStation.class)) {
      mockedFactory
          .when(() -> DriverStation.reportError(anyString(), anyBoolean()))
          .thenCallRealMethod();
      new DebugEntry<Double>(0.0, "test", subsystem);
      new DebugEntry<Double>(0.0, "test", subsystem);
      mockedFactory.verify(() -> DriverStation.reportError(anyString(), anyBoolean()), times(1));
    }
  }

  @Test
  public void testSameNameDifferentSubsystem() {
    if (Robot.isCompetition) {
      return;
    }
    try (MockedStatic<DriverStation> mockedFactory = Mockito.mockStatic(DriverStation.class)) {
      mockedFactory
          .when(() -> DriverStation.reportWarning(anyString(), anyBoolean()))
          .thenCallRealMethod();
      new DebugEntry<Double>(0.0, "test3", subsystem);
      new DebugEntry<Double>(0.0, "test3", subsystem2);
      mockedFactory.verify(() -> DriverStation.reportWarning(anyString(), anyBoolean()), times(0));
    } catch (IllegalArgumentException e) {
    }
  }
}
