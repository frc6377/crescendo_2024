import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.HowdyMath;
import org.junit.jupiter.api.Test;

public class TestTurretSubsystem {
  @Test
  public void TestZeroing() {
    boolean track = true;
    for (double i = 0; i <= 120 / 360d; i += (120 / 360d) * (1d / 10)) {
      double turretPosition =
          TurretSubsystem.encoderPositionsToTurretRotation(
                  (i * Constants.TurretConstants.lowGearCAN_CODER_RATIO) % 1,
                  (i * Constants.TurretConstants.highGearCAN_CODER_RATIO) % 1)
              .getRotations();
      System.out.println("Turret pos:" + i + " guessed pos:" + turretPosition);
      track &= Math.abs(turretPosition - i) < 1e-2;
    }
    assertTrue(track);
  }

  @Test
  public void TestInverseModulous() {
    int testUpTo = 100;
    for (int i = 1; i < testUpTo; i++) {
      // 139 is an arbitraily large prime number
      assertEquals(1, (HowdyMath.inverse_modulus(i, 139) * i) % 139);
    }
  }
}
