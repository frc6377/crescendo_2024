package frc.robot.stateManagement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
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
  private ShooterMode shooterMode = ShooterMode.LONG_RANGE;

  private final Trigger isAmpModeTrigger = new Trigger(() -> placementMode == PlacementMode.AMP);

  private Rotation2d turretRotation = new Rotation2d();
  private Rotation2d robotRotation = new Rotation2d();

  public Rotation2d getTurretRotation() {
    return turretRotation;
  }

  public void setTurretRotation(Rotation2d turretRotation) {
    this.turretRotation = turretRotation;
  }

  // Debug Logging
  private DebugEntry<PlacementMode> placementModeLog =
      new DebugEntry<PlacementMode>(placementMode, "Current Placement Mode", this);
  private DebugEntry<String> shooterModeLog =
      new DebugEntry<String>(shooterMode.toString(), "Current Shooter Mode", this);

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

  public Translation2d speakerPosition() {
    return this.getAllianceColor() == AllianceColor.RED
        ? FieldConstants.RED_SPEAKER
        : FieldConstants.BLUE_SPEAKER;
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
    return isAmpModeTrigger;
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

  public Rotation2d getRobotRotation() {
    return robotRotation;
  }

  public void setRobotRotation(Rotation2d robotRotation) {
    this.robotRotation = robotRotation;
  }

  public ShooterMode getShooterMode() {
    return shooterMode;
  }

  public void setShooterMode(ShooterMode shooterMode) {
    shooterModeLog.log(shooterMode.toString());
    this.shooterMode = shooterMode;
  }

  public Translation2d getLobPosition() {
    if (getAllianceColor() == AllianceColor.BLUE) {
      return FieldConstants.BLUE_LOB_TARGET;
    } else {
      return FieldConstants.RED_LOB_TARGET;
    }
  }

  public Command setShooterMode(ShooterMode targetMode, ShooterMode endMode) {
    return Commands.startEnd(
        () -> this.setShooterMode(targetMode), () -> this.setShooterMode(endMode));
  }
}
