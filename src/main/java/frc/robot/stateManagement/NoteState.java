package frc.robot.stateManagement;

import java.util.Arrays;

public enum NoteState {
  NO_NOTE(0),
  LOADING(10),
  CHAMBER(20),
  READY_TO_FIRE(30),
  READY_TO_PLACE(40);

  private final int integerValue;

  NoteState(final int integerValue) {
    this.integerValue = integerValue;
  }

  public int getAsInt() {
    return integerValue;
  }

  public static NoteState getFromInt(final int integerValue) {
    return Arrays.stream(NoteState.values())
        .filter(v -> v.getAsInt() == integerValue)
        .findFirst()
        .orElse(NO_NOTE);
  }
}
