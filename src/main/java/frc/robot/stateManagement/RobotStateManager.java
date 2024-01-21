package frc.robot.stateManagement;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class RobotStateManager extends SubsystemBase {
  // Alliance Color
  private AllianceColor allianceColor = AllianceColor.UNKNOWN;

  // End Game
  private boolean isEndGame = false;

  // Note State
  private NoteState noteState = NoteState.NO_NOTE;

  // Placement Mode
  private PlacementMode placementMode = PlacementMode.SPEAKER;

  public RobotStateManager() {}

  @Override
  public void periodic() {
    if (allianceColor == AllianceColor.UNKNOWN) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        allianceColor =
            alliance.get().equals(Alliance.Red) ? AllianceColor.RED : AllianceColor.BLUE;
      }
    }
    if (!isEndGame) {
      double matchTimer = DriverStation.getMatchTime();
      boolean isTeleopEnabled = DriverStation.isTeleopEnabled();
      if (matchTimer < 20 && isTeleopEnabled) {
        isEndGame = true;
      }
    }
  }

  // Note State
  public void setNoteState(NoteState noteState) {
    this.noteState = noteState;
  }

  public NoteState getNoteState() {
    return noteState;
  }

  // Placement Mode
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
