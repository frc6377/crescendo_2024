package frc.robot.stateManagement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utilities.DebugEntry;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
  private RangeMode range = RangeMode.SHORT;

  // Debug Logging
  private DebugEntry<PlacementMode> placementModeLog =
      new DebugEntry<PlacementMode>(placementMode, "Current Placement Mode", this);

  public RobotStateManager() {
    endGameStart =
        new Trigger(
            () ->
                (DriverStation.getMatchTime() < Constants.END_GAME_WARNING_TIME
                    && DriverStation.isTeleopEnabled()));
    endGameStart.onTrue(new InstantCommand(() -> isEndGame = true).withName("EndGame Start"));
  }

  @Override
  public void periodic() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      allianceColor = alliance.get().equals(Alliance.Red) ? AllianceColor.RED : AllianceColor.BLUE;
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

  public BooleanSupplier isAmpSupplier() {
    return () -> this.placementMode == PlacementMode.AMP;
  }

  public Trigger isAmpTrigger() {
    return new Trigger(isAmpSupplier());
  }

  public Command setAmpMode() {
    return runOnce(() -> setPlacementMode(PlacementMode.AMP));
  }

  public Command setSpeakerMode() {
    return runOnce(() -> setPlacementMode(PlacementMode.SPEAKER));
  }

  public int getSpeakerCenterTag() {
    if (getAllianceColor() == AllianceColor.RED) {
      return LimelightConstants.SPEAKER_TAG_ID_RED;
    } else if (getAllianceColor() == AllianceColor.BLUE) {
      return LimelightConstants.SPEAKER_TAG_ID_BLUE;
    }
    return -1;
  }

  // Ranging Control

  public void setLongRange() {
    this.range = RangeMode.LONG;
  }

  public void setShortRange() {
    this.range = RangeMode.SHORT;
  }

  public RangeMode getRange() {
    return range;
  }

  private Translation2d allianceCorrect(Translation2d input) {
    if (allianceColor == AllianceColor.RED) {
      return new Translation2d(input.getX(), -input.getY());
    }
    return input;
  }

  public DoubleSupplier getSourceAngle() {
    return () ->
        allianceCorrect(new Translation2d(1, Rotation2d.fromDegrees(-35))).getAngle().getDegrees();
  }

  public DoubleSupplier getSpeakerAngle() {
    return () ->
        allianceCorrect(new Translation2d(1, Rotation2d.fromDegrees(0))).getAngle().getDegrees();
  }
}
