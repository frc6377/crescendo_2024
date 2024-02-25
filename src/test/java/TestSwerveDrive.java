import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.OI;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem.DriveInput;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem.DriveRequest;
import org.junit.jupiter.api.Test;

public class TestSwerveDrive {

  @Test
  public void TestJoystickConditioning() {
    int directions = 100;
    int perDirection = 100;
    int rotations = 50;
    double deadband = 0.02;
    double epsilion = 0.001;

    for (int i = 0; i < directions; i++) {
      Matrix<N2, N1> unitDirction = new Matrix<>(Nat.N2(), Nat.N1());
      unitDirction.set(0, 0, Math.sin(2 * Math.PI * i / directions));
      unitDirction.set(1, 0, Math.cos(2 * Math.PI * i / directions));

      for (int j = 0; j < perDirection; j++) {
        Matrix<N2, N1> translation = unitDirction.times(j / (perDirection + 0.0));
        double actualMagnitude =
            OI.Driver.translationMagnitudeCurve.calculate(
                MathUtil.applyDeadband(j / (perDirection + 0.0), deadband));

        for (int rot = 0; rot < rotations; rot++) {
          DriveInput input = new DriveInput(translation.get(0, 0), translation.get(1, 0), rot);
          DriveRequest req = SwerveSubsystem.joystickCondition(input, deadband);

          assertTrue(Math.abs(req.getMagnitude() - actualMagnitude) < epsilion);
          assertTrue(
              Math.abs(
                      req.alpha()
                          + OI.Driver.rotationCurve.calculate(
                              MathUtil.applyDeadband(rot, deadband)))
                  < epsilion);
        }
      }
    }
  }
}
