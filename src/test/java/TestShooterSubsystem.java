import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.SpeakerConfig;
import java.util.Arrays;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class TestShooterSubsystem {
  // All distances in this test, aside from the negative distance and the extremely large distance,
  // should be part of speakerConfigList! Changes to one should change the other.
  // TODO: Make more sensible
  @Test
  public void testSpeakerConfig() {
    SpeakerConfig testConfig;

    for (int i = 0; i < ShooterSubsystem.speakerConfigList.length; i++) {
      testConfig =
          ShooterSubsystem.calculateShooterSpeeds(
              ShooterSubsystem.speakerConfigList[i].getDistanceInInches());
      Assertions.assertEquals(testConfig, ShooterSubsystem.speakerConfigList[i]);
    }
  }

  public double[] arraySlice(double[] array, int startIndex, int endIndex) {
    double[] slicedArray = Arrays.copyOfRange(array, startIndex, endIndex + 1);
    return slicedArray;
  }
}
