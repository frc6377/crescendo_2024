import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.turretSubsystem.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.HowdyMath;
import org.junit.jupiter.api.Test;

public class TestTurretSubsystem {
  @Test
  public void TestZeroing() {
    double testRange = 130d / 164;
    int numberOfTests = (int) 1000;
    for (double i = 0; i <= testRange; i += testRange * (1d / numberOfTests)) { // 130/164 ?
      double turretPosition =
          TurretSubsystem.encoderPositionsToTurretRotation(
                  (i * Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO) % 1,
                  (i * Constants.TurretConstants.HIGH_GEAR_CAN_CODER_RATIO) % 1)
              .getRotations();
      assertTrue(Math.abs(turretPosition - i) * 360 < 0.1);
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

  @Test
  public void TestInterpolation() {
    TurretSubsystem subsystem =
        new TurretSubsystem(new RobotStateManager(), new VisionSubsystem() {});
    for (double i = 0.2; i < 4; i += 0.2) {
      System.out.print(subsystem.distanceToShootingPitch(i));
      System.out.print(" ");
      System.out.println(i);
    }
  }
}
