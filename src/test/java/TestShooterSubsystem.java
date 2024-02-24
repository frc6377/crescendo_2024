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
    SpeakerConfig speedsPair;
    double[] speedsArray = {0, 0};
    double[][] speakerConfigListTest = {
      {-100, 2550, 1950},
      {0, 2550, 1950},
      {40, 2750, 2150},
      {195, 2950, 2350},
      {290, 2950, 2350},
      {10000, 2950, 2350}
    };
    double[] configSpeeds;

    for (int i = 0; i < speakerConfigListTest.length; i++) {
      speedsPair = ShooterSubsystem.calculateShooterSpeeds(speakerConfigListTest[i][0]);
      speedsArray[0] = speedsPair.getSpeedLeftInRPM();
      speedsArray[1] = speedsPair.getSpeedRightInRPM();
      configSpeeds = arraySlice(speakerConfigListTest[i], 1, 2);
      Assertions.assertArrayEquals(configSpeeds, speedsArray);
    }
  }

  public double[] arraySlice(double[] array, int startIndex, int endIndex) {
    double[] slicedArray = Arrays.copyOfRange(array, startIndex, endIndex + 1);
    return slicedArray;
  }
}
