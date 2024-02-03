package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.Optional;

public class RobotStateManager {
  private static NoteState noteState = NoteState.NO_NOTE;
  private static PlacementMode placementMode = PlacementMode.SPEAKER;

  // Note State
  public static void setNoteState(NoteState state) {
    noteState = state;
  }

  public static NoteState getNoteState() {
    return noteState;
  }

  // Placement Mode
  public static void setPlacementMode(PlacementMode mode) {
    placementMode = mode;
  }

  public static PlacementMode getPlacementMode() {
    return placementMode;
  }

  // Endgame
  public static boolean getIsEndgame() {
    return DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() < Constants.END_GAME_WARNING_TIME;
  }

  public static Trigger getEndgameTrigger() {
    return new Trigger(() -> getIsEndgame());
  }

  // Alliance Color
  public static boolean getIsAllianceRed() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == Alliance.Red : false;
  }

  public static boolean getIsAllianceBlue() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == Alliance.Blue : false;
  }

  // ENUMS
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

  public enum PlacementMode {
    SPEAKER(0),
    AMP(10),
    SOURCE(20);

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
}
