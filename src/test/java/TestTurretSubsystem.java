import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.HowdyMath;
import org.junit.jupiter.api.Test;

public class TestTurretSubsystem {
  @Test
  public void TestZeroing() {
    for (double i = 0; i <= 10; i += 1 / 360d) {
      double turretPosition =
          TurretSubsystem.encoderPositionsToTurretRotation(
                  (i * Constants.TurretConstants.lowGearCAN_CODER_RATIO) % 1,
                  (i * Constants.TurretConstants.highGearCAN_CODER_RATIO) % 1)
              .getRotations();
      assertTrue((turretPosition - i < 1e-8));
    }
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
