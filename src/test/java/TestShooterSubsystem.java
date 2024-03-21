import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem.SpeakerConfig;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class TestShooterSubsystem {
  @Test
  public void testSpeakerConfigEntries() {
    SpeakerConfig testConfig;

    for (int i = 0; i < ShooterSubsystem.speakerConfigList.length; i++) {
      testConfig =
          ShooterSubsystem.calculateShooterSpeeds(
              ShooterSubsystem.speakerConfigList[i].getDistanceInInches());
      Assertions.assertEquals(testConfig, ShooterSubsystem.speakerConfigList[i]);
    }
  }

  @Test
  public void testSpeakerConfigOrder() {
    SpeakerConfig currentConfig;
    SpeakerConfig previousConfig;

    for (int i = 0; i < ShooterSubsystem.speakerConfigList.length; i++) {
      if (i == 0) {
        continue;
      }

      currentConfig = ShooterSubsystem.speakerConfigList[i];
      previousConfig = ShooterSubsystem.speakerConfigList[i - 1];

      Assertions.assertTrue(
          currentConfig.getDistanceInInches() >= previousConfig.getDistanceInInches());
      Assertions.assertTrue(
          currentConfig.getSpeedLeftInRPM() >= previousConfig.getSpeedLeftInRPM());
      Assertions.assertTrue(
          currentConfig.getSpeedRightInRPM() >= previousConfig.getSpeedRightInRPM());
    }
  }

  @Test
  public void testSpeakerConfigOutOfBounds() {
    double belowMinimumDistance = ShooterSubsystem.speakerConfigList[0].getDistanceInInches() - 100;
    double aboveMaximumDistance =
        ShooterSubsystem.speakerConfigList[ShooterSubsystem.speakerConfigList.length - 1]
                .getDistanceInInches()
            + 100;

    SpeakerConfig belowMinimumConfig =
        ShooterSubsystem.calculateShooterSpeeds(belowMinimumDistance);
    SpeakerConfig aboveMaximumConfig =
        ShooterSubsystem.calculateShooterSpeeds(aboveMaximumDistance);

    Assertions.assertEquals(
        belowMinimumConfig.getSpeedLeftInRPM(),
        ShooterSubsystem.speakerConfigList[0].getSpeedLeftInRPM());
    Assertions.assertEquals(
        belowMinimumConfig.getSpeedRightInRPM(),
        ShooterSubsystem.speakerConfigList[0].getSpeedRightInRPM());

    Assertions.assertEquals(
        aboveMaximumConfig.getSpeedLeftInRPM(),
        ShooterSubsystem.speakerConfigList[ShooterSubsystem.speakerConfigList.length - 1]
            .getSpeedLeftInRPM());
    Assertions.assertEquals(
        aboveMaximumConfig.getSpeedRightInRPM(),
        ShooterSubsystem.speakerConfigList[ShooterSubsystem.speakerConfigList.length - 1]
            .getSpeedRightInRPM());
  }
}
