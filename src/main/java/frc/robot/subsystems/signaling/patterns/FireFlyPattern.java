package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

public class FireFlyPattern {
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.GREEN, 5), new PatternNode(RGB.WHITE, 5)
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
