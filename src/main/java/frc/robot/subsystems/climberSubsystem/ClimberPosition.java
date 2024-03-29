package frc.robot.subsystems.climberSubsystem;

import frc.robot.Constants;
import java.util.Arrays;

public enum ClimberPosition {
  DEFAULT(Constants.ClimberConstants.STOWED),
  MAXIMUM(Constants.ClimberConstants.MAXIMUM),
  PREPARE_TO_CLIMB(Constants.ClimberConstants.PREPARE_CLIMB),
  CLIMB(Constants.ClimberConstants.CLIMB);

  private final double angle;

  ClimberPosition(final double angle) {
    this.angle = angle;
  }

  public double getAngle() {
    return angle;
  }

  public static ClimberPosition getFromAngle(final int angle) {
    return Arrays.stream(ClimberPosition.values())
        .filter(v -> v.getAngle() == angle)
        .findFirst()
        .orElse(DEFAULT);
  }
}
