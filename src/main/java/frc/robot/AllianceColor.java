package frc.robot;

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

  public static AllianceColor getFromInt(final int integerValue) {
    return Arrays.stream(AllianceColor.values())
        .filter(v -> v.getAsInt() == integerValue)
        .findFirst()
        .orElse(UNKNOWN);
  }
}
