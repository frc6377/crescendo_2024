package frc.robot.subsystems.turretSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretDataPoint;
import frc.robot.Constants.CommandConstants.LobShotMode;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.stateManagement.ShooterMode;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.HowdyMath;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretCommandFactory {
  final TurretSubsystem subsystem;
  final RobotStateManager RSM;
  final VisionSubsystem vision;
  final Supplier<Rotation2d> rotationSupplier;
  final Supplier<Translation2d> translationSupplier;
  final InterpolatingDoubleTreeMap pitchMap;

  DebugEntry<Double> limelightDistance;

  public TurretCommandFactory(
      TurretSubsystem subsystem,
      RobotStateManager RSM,
      VisionSubsystem visionSubsystem,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<Translation2d> translationSupplier) {
    limelightDistance = new DebugEntry<Double>(0D, "Limelight distance", subsystem);
    this.subsystem = subsystem;
    visionRotation = new DebugEntry<Double>(0d, "Vision Angle", subsystem);
    this.RSM = RSM;
    this.vision = visionSubsystem;
    this.rotationSupplier = rotationSupplier;
    this.translationSupplier = translationSupplier;
    this.pitchMap = new InterpolatingDoubleTreeMap();
    for (TurretDataPoint TDP : TurretConstants.TURRET_DATA) {
      pitchMap.put(TDP.limelightMeters(), TDP.turretAngleRadians());
    }
  }

  public Command stowTurret() {
    if (subsystem == null) return Commands.none();
    return new InstantCommand(
            () ->
                subsystem.setTurretPos(
                    Math.toRadians(Constants.TurretConstants.TURRET_STOWED_ANGLE)))
        .alongWith(
            new InstantCommand(
                () ->
                    subsystem.setPitchPos(
                        Math.toRadians(Constants.TurretConstants.PITCH_STOWED_ANGLE))))
        .withName("StowTurretCommand")
        .asProxy();
  }

  public Command pickup() {
    if (subsystem == null) return Commands.none();
    return new InstantCommand(
            () ->
                subsystem.setTurretPos(
                    Math.toRadians(Constants.TurretConstants.TURRET_PICKUP_ANGLE)))
        .alongWith(
            new InstantCommand(
                () ->
                    subsystem.setPitchPos(
                        Math.toRadians(Constants.TurretConstants.PITCH_PICKUP_ANGLE))))
        .withName("pickup")
        .asProxy();
  }

  /**
   * A command to set the current turret position as true zero.
   *
   * @return a command that sets the current position as true zero
   */
  public Command zeroZeroing() {
    if (subsystem == null) return Commands.none();
    return Commands.runOnce(
            () -> {
              MagnetSensorConfigs cfg = new MagnetSensorConfigs();
              cfg.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
              cfg.withMagnetOffset(0);
              CANcoderConfigurator lowGearCANcoderConfigurator =
                  subsystem.getLowGearCANCoder().getConfigurator();
              CANcoderConfigurator highGearCANcoderConfigurator =
                  subsystem.getHighGearCANCoder().getConfigurator();
              lowGearCANcoderConfigurator.apply(cfg);
              highGearCANcoderConfigurator.apply(cfg);

              final double trueZeroLowGearOffset =
                  subsystem.getLowGearCANCoder().getAbsolutePosition().getValueAsDouble();
              final double trueZeroHighGearOffset =
                  subsystem.getHighGearCANCoder().getAbsolutePosition().getValueAsDouble();

              final double lowGearOffset =
                  trueZeroLowGearOffset
                      - TurretConstants.LOW_GEAR_CAN_CODER_RATIO
                          * TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV;
              final double highGearOffset =
                  trueZeroHighGearOffset
                      - TurretConstants.HIGH_GEAR_CAN_CODER_RATIO
                          * TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV;

              MagnetSensorConfigs newCfgLowGear = new MagnetSensorConfigs();
              newCfgLowGear.withMagnetOffset(lowGearOffset);
              lowGearCANcoderConfigurator.apply(newCfgLowGear);

              MagnetSensorConfigs newCfgHighGear = new MagnetSensorConfigs();
              newCfgHighGear.withMagnetOffset(highGearOffset);
              highGearCANcoderConfigurator.apply(newCfgHighGear);
            },
            subsystem)
        .withName("zeroZeroing");
  }

  public Command zeroTurretCommand() {
    if (subsystem == null) return Commands.none();
    return Commands.runOnce(() -> subsystem.zeroTurret(), subsystem)
        .withName("ZeroTurretCommand")
        .asProxy();
  }

  public Command getAimTurretCommand() {
    if (subsystem == null) return new StartEndCommand(() -> {}, () -> {});

    Supplier<Command> aimCommandSupplier =
        () -> {
          final ShooterMode shooterMode = RSM.getShooterMode();
          switch (shooterMode) {
            case LOB:
              return lobShot();
            case LONG_RANGE:
              return longRangeShot();
            case SHORT_RANGE:
              return shortRangeShot();
            default:
              DriverStation.reportError(
                  String.format("Unknown shooter mode provided (%s)", shooterMode), true);
              return shortRangeShot();
          }
        };

    return Commands.deferredProxy(aimCommandSupplier);
  }

  private Command lobShot() {
    if(CommandConstants.LOB_SHOT_MODE == LobShotMode.ODOMETRY_BASED){
      return subsystem.startEnd(
          () -> {
            subsystem.setPositionErrorSupplier(
                () ->
                    odometryPointing(RSM.getLobPosition()).getRotations() + subsystem.getTurretPos());
            subsystem.setPitchPos(TurretConstants.LOB_PITCH);
          },
          () -> {});
    }

    // This handles both fixed and the error case of unknown lob shot mode
    if(CommandConstants.LOB_SHOT_MODE != LobShotMode.FIXED){
      DriverStation.reportError("Unknown Lob shot mode set, defaulting to fixed", true);
    }
    return subsystem.startEnd(() -> subsystem.setTurretPos(0),()->{});
  }

  public Command shortRangeShot() {
    return subsystem.startEnd(
        () -> {
          subsystem.setTurretPos(Math.toRadians(00));
          subsystem.setPitchPos(Math.toRadians(37));
        },
        () -> {});
  }

  public Command longRangeShot() {
    return subsystem.run(
        () -> {
          if (Constants.enabledSubsystems.turretRotationEnabled) {
            visionTracking();
          }
          if (Constants.enabledSubsystems.turretPitchEnabled) {
            double distance = distanceEstimateMeters();
            limelightDistance.log(distance);
            subsystem.setPitchPos(pitchMap.get(distance));
          }
        });
  }

  private void visionTracking() {
    visionTracking(() -> speakerPointing());
  }

  private void visionTracking(Supplier<Rotation2d> searchingBehavior) {
    visionTracking(getSpeakerTag(), searchingBehavior);
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

  /**
   * @param targetTag the tag to search towards
   * @param searchingBehavior target rotation in degrees
   */
  private void visionTracking(int targetTag, Supplier<Rotation2d> searchingBehavior) {
    subsystem.setPositionErrorSupplier(
        () -> {
          double errDegrees = vision.getTurretYaw(targetTag);
          if (Double.isNaN(errDegrees)) {
            return searchingBehavior.get().getRotations() + subsystem.getTurretPos();
          }
          double err = Units.degreesToRotations(errDegrees);
          return err;
        });
  }

  public Command idleTurret() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .run(
            () -> {
              subsystem.setTurretPos(0);
              moveToBottomOfTravel();
            })
        .withName("idleTurret");
  }

  public Command pinTurret() {
    return subsystem.run(
        () -> {
          if (subsystem.turretAtSetPoint(TurretConstants.ALLOWED_PIN_ERROR)) {
            moveToBottomOfTravel();
            // Effectivly disables the rotation motor
            subsystem.setPositionErrorSupplier(() -> 0);
          } else {
            subsystem.setTurretPos(0);
          }
        });
  }

  public Command testTurretCommand(DoubleSupplier degrees) {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runEnd(
            () -> {
              limelightDistance.log(vision.getDistance(getSpeakerTag()));
              subsystem.setPitchPos(Math.toRadians(degrees.getAsDouble()));
            },
            subsystem::stopTurret)
        .withName("TestTurret")
        .asProxy();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }

  private void moveToBottomOfTravel() {
    subsystem.setTurretPos(0);
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

  public Trigger isReady() {
    if (subsystem == null) return null;
    return new Trigger(
        () ->
            subsystem.pitchAtSetpoint() && subsystem.turretAtSetPoint(Rotation2d.fromDegrees(2.5)));
  }
}
