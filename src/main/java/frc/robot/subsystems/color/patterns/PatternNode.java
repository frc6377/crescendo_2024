package frc.robot.subsystems.color.patterns;

import frc.robot.subsystems.color.RGB;

public class PatternNode {
  public final RGB color;
  public final int repeat;

  public PatternNode(RGB color, int repeat) {
    this.color = color;
    this.repeat = repeat;
  }
}
