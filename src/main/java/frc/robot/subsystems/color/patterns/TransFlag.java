package frc.robot.subsystems.color.patterns;

import frc.robot.subsystems.color.RGB;

public class TransFlag {
  public static int numberOfLEDS;
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.BLACK, 1),
    new PatternNode(RGB.HOWDY_BLUE, 4),
    new PatternNode(RGB.PINK, 4),
    new PatternNode(RGB.WHITE, 4),
    new PatternNode(RGB.PINK, 4),
    new PatternNode(RGB.HOWDY_BLUE, 4),
  };
  private static int patternLength;

  static {
    for (PatternNode p : pattern) {
      patternLength += p.repeat;
    }
  }

  public static int getPatternLength() {
    return patternLength;
  }

  public static PatternNode[] getPattern() {
    return pattern;
  }
}
