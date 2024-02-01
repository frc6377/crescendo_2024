import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.Pair;
import java.util.Arrays;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestShooterSubsystem {
  private ShooterSubsystem shooterSubsystemTest;

  @BeforeEach
  public void initialize() {
    shooterSubsystemTest = new ShooterSubsystem();
  }

  // All distances in this test, aside from the negative distance and the extremely large distance,
  // should be part of speakerConfigList! Changes to one should change the other.
  // TODO: Make more sensible
  @Test
  public void TestSpeakerConfig() {
    Pair<Double, Double> speedsPair;
    double[] speedsArray = {0, 0};
    double[][] speakerConfigListTest = {
      {-100, 450, 250},
      {0, 450, 250},
      {40, 550, 350},
      {195, 750, 500},
      {290, 1000, 700},
      {10000, 1000, 700}
    };
    double[] configSpeeds;

    for (int i = 0; i < speakerConfigListTest.length; i++) {
      speedsPair = ShooterSubsystem.calculateShooterSpeeds(speakerConfigListTest[i][0]);
      speedsArray[0] = speedsPair.getFirst();
      speedsArray[1] = speedsPair.getSecond();
      configSpeeds = arraySlice(speakerConfigListTest[i], 1, 2);
      Assertions.assertArrayEquals(configSpeeds, speedsArray);
    }
  }

  public double[] arraySlice(double[] array, int startIndex, int endIndex) {
    double[] slicedArray = Arrays.copyOfRange(array, startIndex, endIndex + 1);
    return slicedArray;
  }
}
