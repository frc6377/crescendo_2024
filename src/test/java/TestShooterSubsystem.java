import frc.robot.subsystems.ShooterSubsystem;
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
    double speed;
    double[][] speakerConfigListTest = {
      {-100, 450},
      {0, 450},
      {40, 550},
      {195, 750},
      {290, 1000},
      {10000, 1000}
    };

    for (int i = 0; i < speakerConfigListTest.length; i++) {
      speed = ShooterSubsystem.calculateShooterSpeed(speakerConfigListTest[i][0]);
      Assertions.assertEquals(speakerConfigListTest[i][1], speed);
      System.out.println(speakerConfigListTest[i][1] + " " + speed);
    }
  }
}
