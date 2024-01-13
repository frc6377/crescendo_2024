package frc.robot.subsystems.color.patterns;

import frc.robot.Constants;
import frc.robot.subsystems.color.RGB;

public class FireFlyPattern {
  public static int numberOfLEDS = Constants.LED_COUNT;
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
