package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class HowdyMath {
  /**
   * Checks if the value is in the range. Inclusive on both sides
   *
   * @param value the value to check
   * @param min the minimum value
   * @param max the maximum value
   * @return if the value is between the max and min values
   */
  public static boolean inRange(double value, double min, double max) {
    return value <= max && value >= min;
  }

  public static Transform2d pose2dToTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  public static Rotation2d getAngleToTarget(Translation2d currentPosition, Translation2d target) {
    final Translation2d delta = currentPosition.minus(target);
    return new Rotation2d(delta.getX(), delta.getY());
  }

  /**
   * Returns modulo inverse of a with respect to m using extended Euclid Algorithm. Refer below post
   * for details:
   *
   * <p>ax mod m = 1
   *
   * @param a the scalar to use
   * @param m the value to modulo by
   * @retrun the modulo inverse
   */
  public static int inverse_modulus(int a, int m) {
    int m0 = m, t, q;
    int x0 = 0, x1 = 1;
    if (m == 1) return 0;

    // Apply extended Euclid Algorithm
    while (a > 1) {
      // q is quotient
      q = a / m;

      t = m;

      // m is remainder now, process
      // same as euclid's algo
      m = a % m;
      a = t;

      t = x0;

      x0 = x1 - q * x0;

      x1 = t;
    }

    // Make x1 positive
    if (x1 < 0) x1 += m0;

    return x1;
  }
}
