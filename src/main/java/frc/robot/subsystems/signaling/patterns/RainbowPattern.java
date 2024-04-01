package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

public class RainbowPattern {
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.RED, 1),
    new PatternNode(RGB.ORANGE, 1),
    new PatternNode(RGB.YELLOW, 1),
    new PatternNode(new RGB(255 / 2, 255, 0), 1),
    new PatternNode(RGB.GREEN, 1),
    new PatternNode(new RGB(0, 255, 255 / 2), 1),
    new PatternNode(new RGB(0, 255, 255), 1),
    new PatternNode(new RGB(0, 255 / 2, 255), 1),
    new PatternNode(RGB.BLUE, 1),
    new PatternNode(RGB.PURPLE, 1),
    new PatternNode(new RGB(255, 0, 255), 1),
    new PatternNode(new RGB(255, 0, 255 / 2), 1)
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
