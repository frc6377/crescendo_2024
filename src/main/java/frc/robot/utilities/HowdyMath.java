package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class HowdyMath {

  public static Rotation2d getAngleToTarget(Translation2d currentPosition, Translation2d target) {
    final Translation2d delta = currentPosition.minus(target);
    return new Rotation2d(delta.getX(), delta.getY());
  }

  public static double[] getPidgeonTranslation(double initVel, double measAccel, double deltaT) {
    double d[] = new double[2];
    double vel = (initVel + measAccel * deltaT);
    double pos = .5 * measAccel * Math.pow(deltaT, 2) + vel * deltaT;
    d[0] = pos;
    d[1] = vel;
    return d;
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
