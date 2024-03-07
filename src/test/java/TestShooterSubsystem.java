import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem.SpeakerConfig;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class TestShooterSubsystem {
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
}
