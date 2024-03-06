import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem.SpeakerConfig;
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
      {-100, 2350, 1950},
      {0, 2350, 1950},
      {40, 350, 350},
      {195, 500, 500},
      {290, 700, 700},
      {10000, 700, 700}
    };
    ;
    double[] configSpeeds;

    for (int i = 0; i < speakerConfigListTest.length; i++) {
      speedsPair = ShooterSubsystem.calculateShooterSpeeds(speakerConfigListTest[i][0]);
      speedsArray[0] = speedsPair.getSpeedLeftInRPM();
      speedsArray[1] = speedsPair.getSpeedRightInRPM();
      configSpeeds = arraySlice(speakerConfigListTest[i], 1, 2);
      Assertions.assertArrayEquals(configSpeeds, speedsArray);
    }
  }

  private double[] toSpeedPair(SpeakerConfig cfg) {
    return new double[] {cfg.getSpeedLeftInRPM(), cfg.getSpeedRightInRPM()};
  }

  public double[] arraySlice(double[] array, int startIndex, int endIndex) {
    double[] slicedArray = Arrays.copyOfRange(array, startIndex, endIndex + 1);
    return slicedArray;
  }
}
