package frc.robot;

import java.util.Arrays;

public enum PlacementMode {
  SPEAKER(0),
  AMP(10);

  private final int integerValue;

  PlacementMode(final int integerValue) {
    this.integerValue = integerValue;
  }

  public int getAsInt() {
    return integerValue;
  }

  public static PlacementMode getFromInt(final int integerValue) {
    return Arrays.stream(PlacementMode.values())
        .filter(v -> v.getAsInt() == integerValue)
        .findFirst()
        .orElse(SPEAKER);
  }
}
