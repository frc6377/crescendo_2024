import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.robot.subsystems.vision.PhotonSubsystem;
import frc.robot.utilities.HowdyMath;
import frc.robot.utilities.HowdyMath.Equation3d;
import java.util.LinkedList;
import org.junit.jupiter.api.Test;

public class TestMath {
  private static double epsilon = 0.05;

  @Test
  public void testDistanceBasedOdometry() {
    int checks = 1;
    long start = System.nanoTime();
    for (int i = 0; i < checks; i++) {
      test();
    }
    long end = System.nanoTime();
    System.out.println((end - start) / checks / 1e9);
  }

  private void test() {
    Pose2d initialPosition = new Pose2d(2, 3, new Rotation2d());
    LinkedList<Pair<Translation2d, Measure<Distance>>> circles = new LinkedList<>();
    circles.push(Pair.of(new Translation2d(1, 1), Meters.of(1)));
    // circles.push(Pair.of(new Translation2d(2, 3), Meters.of(2)));

    LinkedList<AprilTag> aprilTags = new LinkedList<>();
    LinkedList<Pair<Integer, Measure<Distance>>> aprilTagCircles = new LinkedList<>();

    for (int i = 0; i < circles.size(); i++) {
      Translation2d center = circles.get(i).getFirst();
      aprilTags.push(
          new AprilTag(
              i, new Pose3d(new Translation3d(center.getX(), center.getY(), 0), new Rotation3d())));
      aprilTagCircles.add(Pair.of(i, circles.get(i).getSecond()));
    }

    AprilTagFieldLayout layout = new AprilTagFieldLayout(aprilTags, 1, 1);

    Translation2d position =
        PhotonSubsystem.distancesBasedEstimate(aprilTagCircles, layout, initialPosition);
    System.out.println(position);
  }

  @Test
  public void testGradientDescent() {
    Pair<Double, Double> center = new Pair<>(2.1d, 2d);
    QuadraticEquation equation = new QuadraticEquation(1, 2, center);
    Pair<Double, Double> p =
        HowdyMath.gradientDescentOfKnownEquation(equation, new Pair<Double, Double>(20d, 20d), 5);
    assertTrue(
        p.getFirst() * p.getFirst()
                + p.getSecond() * p.getSecond()
                - center.getFirst() * center.getFirst()
                - center.getSecond() * center.getSecond()
            < epsilon);
  }

  private class QuadraticEquation implements Equation3d {
    private double a = 0;
    private double b = 0;
    private Pair<Double, Double> c;

    public QuadraticEquation(double a, double b, Pair<Double, Double> c) {
      this.a = a;
      this.b = b;
      this.c = c;
    }

    @Override
    public double calculateZ(Pair<Double, Double> point2) {
      Pair<Double, Double> point =
          new Pair<>(point2.getFirst() - c.getFirst(), point2.getSecond() - c.getSecond());
      return a * (point.getFirst() * point.getFirst() + point.getSecond() * point.getSecond()) + b;
    }

    @Override
    public Pair<Double, Double> calculateZDerivative(Pair<Double, Double> point2) {
      Pair<Double, Double> point =
          new Pair<>(point2.getFirst() - c.getFirst(), point2.getSecond() - c.getSecond());
      return new Pair<>(a * point.getFirst() * 2, a * point.getSecond() * 2);
    }

    @Override
    public Pair<Double, Double> calculateZSecondDerivative(Pair<Double, Double> point) {
      return new Pair<Double, Double>(2d * a, 2d * a);
    }
  }
}
