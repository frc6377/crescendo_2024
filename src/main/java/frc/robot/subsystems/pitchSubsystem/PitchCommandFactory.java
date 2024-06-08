package frc.robot.subsystems.pitchSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.CommandConstants.LobShotMode;
import frc.robot.Constants.DevTools;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretDataPoint;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.stateManagement.ShooterMode;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.HowdyMath;
import frc.robot.utilities.TunableNumber;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PitchCommandFactory {
  final PitchSubsystem subsystem;
  final RobotStateManager RSM;
  final VisionSubsystem vision;
  final Supplier<Rotation2d> rotationSupplier;
  final Supplier<Translation2d> translationSupplier;
  final InterpolatingDoubleTreeMap pitchMap;

  DebugEntry<Boolean> isReadyLog;
  DebugEntry<Double> limelightDistance;
  TunableNumber shooterPitch;

  public PitchCommandFactory(
      PitchSubsystem subsystem,
      RobotStateManager RSM,
      VisionSubsystem visionSubsystem,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<Translation2d> translationSupplier) {
    this.subsystem = subsystem;
    if (subsystem != null) {
      limelightDistance = new DebugEntry<Double>(0D, "Limelight distance", subsystem);
      visionRotation = new DebugEntry<Double>(0d, "Vision Angle", subsystem);
      shooterPitch = new TunableNumber("Shooter Pitch", 0, subsystem);
    }
    isReadyLog = new DebugEntry<Boolean>(false, "is ready", subsystem);

    this.RSM = RSM;
    this.vision = visionSubsystem;
    this.rotationSupplier = rotationSupplier;
    this.translationSupplier = translationSupplier;
    this.pitchMap = new InterpolatingDoubleTreeMap();
    for (TurretDataPoint TDP : TurretConstants.TURRET_DATA) {
      pitchMap.put(TDP.limelightMeters(), TDP.turretAngleRadians());
    }
  }

  public Command stowPitch() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runOnce(
            () -> {
              subsystem.setPitchPos(Math.toRadians(TurretConstants.PITCH_STOWED_ANGLE));
            })
        .withName("StowPitchCommand")
        .asProxy();
  }

  public Command pickup() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runOnce(
            () -> {
              subsystem.setPitchPos(Math.toRadians(TurretConstants.PITCH_PICKUP_ANGLE));
            })
        .withName("pitchPickup")
        .asProxy();
  }

  public Command getAimPitchCommand() {
    if (subsystem == null) return new StartEndCommand(() -> {}, () -> {});

    Supplier<Command> aimPitchCommandSupplier =
        () -> {
          final ShooterMode shooterMode = RSM.getShooterMode();
          switch (shooterMode) {
            case LOB:
              return shortRangeShot();
            case LONG_RANGE:
              return longRangeShot();
            case SHORT_RANGE:
              return shortRangeShot();
            case NO_ODOM:
              return subsystem
                  .run(() -> subsystem.setPitchPos(pitchMap.get(distanceEstimateMeters())))
                  .withName("No Rotation")
                  .asProxy();
            default:
              DriverStation.reportError(
                  String.format("Unknown shooter mode provided (%s)", shooterMode), true);
              return shortRangeShot();
          }
        };

    return Commands.deferredProxy(aimPitchCommandSupplier).withName("getAimPitchCommand");
  }

  private Command lobShot() {
    if (CommandConstants.LOB_SHOT_MODE == LobShotMode.ODOMETRY_BASED) {
      return subsystem.startEnd(
          () -> {
            subsystem.setPitchPos(TurretConstants.LOB_PITCH);
          },
          () -> {});
    }

    // This handles both fixed and the error case of unknown lob shot mode
    if (CommandConstants.LOB_SHOT_MODE != LobShotMode.FIXED) {
      DriverStation.reportError("Unknown Lob shot mode set, defaulting to fixed", true);
    }
    return Commands.none();
  }

  public Command shortRangeShot() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setPitchPos(Math.toRadians(37));
            },
            () -> {})
        .withName("shortRangeShotPitch")
        .asProxy();
  }

  public Command longRangeShot() {
    return longRangeShot(() -> speakerPointing())
        .withName("long shot with default search behavior");
  }

  public Command longRangeShot(Supplier<Rotation2d> searchBehavior) {
    if (subsystem == null) return Commands.none();
    return subsystem
        .run(
            () -> {
              if (Constants.enabledSubsystems.turretPitchEnabled) {
                double distance = distanceEstimateMeters();
                limelightDistance.log(distance);
                if (DevTools.ShooterLinerizing) {
                  subsystem.setPitchPos(
                      Units.degreesToRadians(SmartDashboard.getNumber("Set Shooter Pitch", 0)));
                } else {
                  subsystem.setPitchPos(pitchMap.get(distance));
                }
              }
            })
        .withName("longRangeShot")
        .asProxy();
  }

  private int getSpeakerTag() {
    return RSM.getAllianceColor() == AllianceColor.RED
        ? LimelightConstants.SPEAKER_TAG_ID_RED
        : LimelightConstants.SPEAKER_TAG_ID_BLUE;
  }

  private double distanceEstimateMeters() {
    double visionDistance = vision.getDistance(getSpeakerTag());
    if (Double.isNaN(visionDistance)) {
      return odometryDistance();
    }
    return visionDistance;
  }

  private double odometryDistance() {
    return positionRelativeToSpeaker().getNorm();
  }

  DebugEntry<Double> visionRotation;

  public Command idleTurret() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .run(
            () -> {
              moveToBottomOfTravel();
            })
        .withName("idleTurret")
        .asProxy();
  }

  public Command testTurretCommand(DoubleSupplier degrees) {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runEnd(
            () -> {
              subsystem.setPitchPos(Math.toRadians(degrees.getAsDouble()));
            },
            subsystem::stopPitch)
        .withName("TestPitch")
        .asProxy();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(
        Commands.parallel(subsystem.runOnce(() -> {}), defaultCommand)
            .withName(defaultCommand.getName()));
  }

  public Command[] getCommands() {
    ArrayList<Command> cmds = new ArrayList<Command>();
    cmds.add(stowPitch());
    cmds.add(pickup());
    cmds.add(getAimPitchCommand());
    cmds.add(idleTurret());
    cmds.add(this.longRangeShot());
    cmds.add(this.longRangeShot(() -> new Rotation2d()));
    cmds.add(this.shortRangeShot());
    return cmds.toArray(new Command[cmds.size()]);
  }

  private void moveToBottomOfTravel() {
    if (subsystem.getPitch() > 0.05) {
      subsystem.setPitchPos(0);
    } else {
      subsystem.holdPosition();
    }
  }

  private Translation2d positionRelativeToSpeaker() {
    return translationSupplier.get().minus(RSM.speakerPosition());
  }

  private Rotation2d speakerPointing() {
    return odometryPointing(RSM.speakerPosition());
  }

  private Rotation2d odometryPointing(Translation2d targetPosition) {
    final Rotation2d targetTurretAngleRelToField =
        HowdyMath.getAngleToTarget(translationSupplier.get(), targetPosition);
    final Rotation2d targetTurretAngleRelToRobot =
        targetTurretAngleRelToField.minus(rotationSupplier.get());
    return targetTurretAngleRelToRobot.times(-1);
  }

  public Trigger isReadyTrigger() {
    if (subsystem == null) return new Trigger(() -> true);
    return new Trigger(this::isReadyBoolean);
  }

  public boolean isReadyBoolean() {
    if (subsystem == null) return true;
    boolean ready = subsystem.pitchAtSetpoint();
    isReadyLog.log(ready);
    return ready;
  }

  public enum SearchingBehavior {
    ODOMETRY,
    STAY_STILL;
  }
}
