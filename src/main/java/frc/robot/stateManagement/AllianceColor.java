package frc.robot.stateManagement;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import java.util.Arrays;

public enum AllianceColor {
  UNKNOWN(0),
  RED(10),
  BLUE(20);

  private final int integerValue;

  AllianceColor(final int integerValue) {
    this.integerValue = integerValue;
  }

  public int getAsInt() {
    return integerValue;
  }

  public Translation2d getSpeakerLocation() {
    switch (this) {
      case BLUE:
        return FieldConstants.BLUE_SPEAKER;
      case RED:
        return FieldConstants.RED_SPEAKER;
      default:
        return new Translation2d(0, 0);
    }
  }

  public static AllianceColor getFromInt(final int integerValue) {
    return Arrays.stream(AllianceColor.values())
        .filter(v -> v.getAsInt() == integerValue)
        .findFirst()
        .orElse(UNKNOWN);
  }
}
