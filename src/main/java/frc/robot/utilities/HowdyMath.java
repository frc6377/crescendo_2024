package frc.robot.utilities;

public class HowdyMath {
  // Returns modulo inverse of a
  // with respect to m using extended
  // Euclid Algorithm. Refer below post for details:
  //
  public static double inverse_modulus(double a, double m) {
    double m0 = m, t, q;
    double x0 = 0, x1 = 1;
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
