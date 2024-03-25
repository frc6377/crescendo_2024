package frc.robot.utilities;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class HowdyMath {
  public static Pair<Double, Double> gradientDescentOfKnownEquation(
      Equation3d function, Pair<Double, Double> initialGuess, int iterations) {
    Pair<Double, Double> x = initialGuess;
    for (int i = 0; i < iterations; i++) {
      x =
          subtractPairs(
              x,
              pieceWiseDivide(
                  function.calculateZDerivative(x), function.calculateZSecondDerivative(x)));
      System.out.printf(
          "%7.2f, %s, %s, %s%n",
          function.calculateZ(x),
          formatPair(x),
          formatPair(function.calculateZDerivative(x)),
          formatPair(function.calculateZSecondDerivative(x)));
    }
    return x;
  }

  public static String formatPair(Pair<Double, Double> a) {
    return String.format("(%7.2f, %7.2f)", a.getFirst(), a.getSecond());
  }

  public static double pairMagnitude(Pair<Double, Double> input) {
    return Math.hypot(input.getFirst(), input.getSecond());
  }

  public static Pair<Double, Double> subtractPairs(Pair<Double, Double> a, Pair<Double, Double> b) {
    return new Pair<Double, Double>(a.getFirst() - b.getFirst(), a.getSecond() - b.getSecond());
  }

  public static Pair<Double, Double> pieceWiseDivide(
      Pair<Double, Double> a, Pair<Double, Double> b) {
    return new Pair<Double, Double>(a.getFirst() / b.getFirst(), a.getSecond() / b.getSecond());
  }

  public interface Equation3d {
    public double calculateZ(Pair<Double, Double> point);

    public Pair<Double, Double> calculateZDerivative(Pair<Double, Double> point);

    public Pair<Double, Double> calculateZSecondDerivative(Pair<Double, Double> point);
  }

  public static Rotation2d getAngleToTarget(Translation2d currentPosition, Translation2d target) {
    Translation2d delta = currentPosition.minus(target);
    return new Rotation2d(delta.getX(), delta.getY());
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

  public static Pair<Double, Double> addPairs(Pair<Double, Double> a, Pair<Double, Double> b) {
    return new Pair<Double, Double>(a.getFirst() + b.getFirst(), a.getSecond() + b.getSecond());
  }

  public static Pair<Double, Double> poseToPair(Pose2d previousRobotPosition) {
    return Pair.of(previousRobotPosition.getX(), previousRobotPosition.getY());
  }

  public static Translation2d translation2dFromPair(Pair<Double, Double> robotPosition) {
    return new Translation2d(robotPosition.getFirst(), robotPosition.getSecond());
  }
}
