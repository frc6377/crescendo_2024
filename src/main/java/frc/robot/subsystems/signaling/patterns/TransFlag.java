package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

public class TransFlag {
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.HOWDY_BLUE, 1),
    new PatternNode(RGB.PINK, 1),
    new PatternNode(RGB.WHITE, 1),
    new PatternNode(RGB.PINK, 1),
    new PatternNode(RGB.HOWDY_BLUE, 1),
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
