package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;

public class HowdyMath {

  public static Supplier<Rotation2d> getAngleToTargetContinous(
      Supplier<Translation2d> robotPosition, Translation2d target) {
    return () -> {
      Translation2d delta = robotPosition.get().minus(target);
      return new Rotation2d(delta.getX(), delta.getY());
    };
  }

  // Returns modulo inverse of a
  // with respect to m using extended
  // Euclid Algorithm. Refer below post for details:
  //
  // ax mod m = 1
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
