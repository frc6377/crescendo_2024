package frc.robot.subsystems.signaling.patterns;

import frc.robot.Constants;
import frc.robot.subsystems.signaling.RGB;

public class BIFlag {
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.BLACK, 2),
    new PatternNode(RGB.PINK, 3),
    new PatternNode(RGB.PURPLE, 3),
    new PatternNode(RGB.BLUE, 3),
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

  public static PatternNode[] getColors(int step) {
    int initalStep = step % pattern.length;
    PatternNode[] fullPattern = new PatternNode[Constants.NUMBER_OF_LEDS];
    for (int i = 0; i < Constants.NUMBER_OF_LEDS; i++) {
      fullPattern[i] = pattern[(i + initalStep) % pattern.length];
    }
    return fullPattern;
  }
}
