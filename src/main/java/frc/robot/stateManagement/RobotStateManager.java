package frc.robot.stateManagement;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utilities.DebugEntry;
import java.util.Optional;

public class RobotStateManager extends SubsystemBase {
  // Alliance Color
  private AllianceColor allianceColor = AllianceColor.UNKNOWN;

  // End Game
  private boolean isEndGame = false;
  private Trigger endGameStart;

  // Note State
  private NoteState noteState = NoteState.NO_NOTE;

  // Placement Mode
  private PlacementMode placementMode = PlacementMode.SPEAKER;

  // Debug Logging
  private DebugEntry<PlacementMode> placementModeLog =
      new DebugEntry<PlacementMode>(placementMode, "Current Placement Mode", this);

  public RobotStateManager() {
    endGameStart =
        new Trigger(
            () ->
                (DriverStation.getMatchTime() < Constants.END_GAME_WARNING_TIME
                    && DriverStation.isTeleopEnabled()));
  }

  @Override
  public void periodic() {
    if (allianceColor == AllianceColor.UNKNOWN) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        allianceColor =
            alliance.get().equals(Alliance.Red) ? AllianceColor.RED : AllianceColor.BLUE;
      }
    }
    endGameStart.onTrue(new InstantCommand(() -> isEndGame = true).withName("EndGame Start"));
  }

  // Note State
  public void setNoteState(NoteState noteState) {
    this.noteState = noteState;
  }

  public NoteState getNoteState() {
    return noteState;
  }

  // Placement Mode
  public void switchPlacementMode() {
    placementMode =
        placementMode == PlacementMode.SPEAKER ? PlacementMode.AMP : PlacementMode.SPEAKER;
    placementModeLog.log(placementMode);
  }

  public void setPlacementMode(PlacementMode placementMode) {
    this.placementMode = placementMode;
  }

  public PlacementMode getPlacementMode() {
    return placementMode;
  }

  // Alliance Color
  public AllianceColor getAllianceColor() {
    return allianceColor;
  }
}
