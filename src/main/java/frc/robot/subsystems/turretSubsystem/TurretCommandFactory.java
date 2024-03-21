package frc.robot.subsystems.turretSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.config.TurretZeroConfig;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RangeMode;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.HowdyMath;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretCommandFactory {
  final TurretSubsystem subsystem;
  final RobotStateManager RSM;
  final VisionSubsystem vision;
  final Supplier<Rotation2d> rotationSupplier;
  final Supplier<Translation2d> translationSupplier;

  public TurretCommandFactory(
      TurretSubsystem subsystem,
      RobotStateManager RSM,
      VisionSubsystem visionSubsystem,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<Translation2d> translationSupplier) {
    this.subsystem = subsystem;
    this.RSM = RSM;
    this.vision = visionSubsystem;
    this.rotationSupplier = rotationSupplier;
    this.translationSupplier = translationSupplier;
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

              DynamicRobotConfig dynamicConfig = new DynamicRobotConfig();
              dynamicConfig.saveTurretZero(new TurretZeroConfig(lowGearOffset, highGearOffset));
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
    return Commands.either(
        shortRangeShot(), longRangeShot(), () -> RSM.getRange() == RangeMode.SHORT);
  }

  public Command shortRangeShot() {
    return subsystem.startEnd(
        () -> {
          if (Constants.enabledSubsystems.turretRotationEnabled)
            subsystem.setTurretPos(Math.toRadians(00));
          if (Constants.enabledSubsystems.turretPitchEnabled)
            subsystem.setPitchPos(Math.toRadians(40));
        },
        () -> {});
  }

  public Command longRangeShot() {
    double angleToTarget = 0.35;
    DoubleSupplier targetAngle =
        () -> RSM.getAllianceColor() == AllianceColor.RED ? angleToTarget : -angleToTarget;
    return subsystem.startEnd(
        () -> {
          if (Constants.enabledSubsystems.turretRotationEnabled) {
            if (CommandConstants.USE_VISION_TARGETING) {
              visionTracking();
            } else {
              subsystem.setTurretPos(targetAngle.getAsDouble());
            }
          }
          if (Constants.enabledSubsystems.turretPitchEnabled) {
            subsystem.setPitchPos(Math.toRadians(22.5));
          }
        },
        () -> {});
  }

  private void visionTracking() {
    visionTracking(this::odometryPointing);
  }

  private void visionTracking(Supplier<Rotation2d> searchingBehavior) {
    visionTracking(
        RSM.getAllianceColor() == AllianceColor.RED
            ? LimelightConstants.SPEAKER_TAG_ID_RED
            : LimelightConstants.SPEAKER_TAG_ID_BLUE,
        searchingBehavior);
  }

  /**
   * @param targetTag the tag to search towards
   * @param searchingBehavior target rotation in degrees
   */
  private void visionTracking(int targetTag, Supplier<Rotation2d> searchingBehavior) {
    subsystem.setPositionErrorSupplier(
        () -> {
          double errDegrees = vision.getTurretYaw(targetTag);
          if (Double.isNaN(errDegrees)) {
            return searchingBehavior.get().getRotations();
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
              moveToBottomOfTravel();
            })
        .withName("idleTurret");
  }

  public Command pinTurret() {
    return subsystem.run(
        () -> {
          if (subsystem.turretAtSetPoint(TurretConstants.PIN_EPSILION)) {
            moveToBottomOfTravel();
            // Effectivly disables the rotation motor
            subsystem.setPositionErrorSupplier(() -> 0);
          } else {
            subsystem.setTurretPos(0);
          }
        });
  }

  public Command testTurretCommand(double degrees) {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runEnd(() -> subsystem.setPitchPos(Math.toRadians(degrees)), subsystem::stopTurret)
        .withName("TestTurret")
        .asProxy();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }

  public Command logCurrentAngle() {
    return subsystem
        .runOnce(() -> System.out.println(subsystem.getTurretPos()))
        .ignoringDisable(true);
  }

  private void moveToBottomOfTravel() {
    subsystem.setTurretPos(0);
    if (subsystem.getPitch() > 0.05) {
      subsystem.setPitchPos(0);
    } else {
      subsystem.holdPosition();
    }
  }

  private Rotation2d odometryPointing() {
    final Translation2d targetPosition =
        RSM.getAllianceColor() == AllianceColor.RED
            ? FieldConstants.RED_SPEAKER
            : FieldConstants.BLUE_SPEAKER;
    final Rotation2d targetTurretAngleRelToField =
        HowdyMath.getAngleToTarget(translationSupplier.get(), targetPosition);
    final Rotation2d targetTurretAngleRelToRobot =
        targetTurretAngleRelToField.minus(rotationSupplier.get());
    return targetTurretAngleRelToRobot;
  }
}
