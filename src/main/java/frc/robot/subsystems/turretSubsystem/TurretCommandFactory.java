package frc.robot.subsystems.turretSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.config.TurretZeroConfig;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.HowdyMath;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretCommandFactory {
  final TurretSubsystem subsystem;
  final VisionSubsystem visionSubsystem;
  final Supplier<Pose2d> robotPosition;
  final RobotStateManager RSM;

  public TurretCommandFactory(
      TurretSubsystem subsystem,
      Supplier<Pose2d> robotPosition,
      RobotStateManager robotStateManager) {
    this.subsystem = subsystem;
    this.visionSubsystem = subsystem.getVisionSubsystem();
    this.robotPosition = robotPosition;
    RSM = robotStateManager;
  }

  public Command stowTurret() {
    if (subsystem == null) return Commands.none();
    return new InstantCommand(
            () -> {
              subsystem.setTurretPos(Math.toRadians(Constants.TurretConstants.TURRET_STOWED_ANGLE));
              subsystem.setPitchPos(Math.toRadians(Constants.TurretConstants.PITCH_STOWED_ANGLE));
            })
        .withName("StowTurretCommand")
        .asProxy();
  }

  public Command pickup() {
    if (subsystem == null) return Commands.none();
    return new InstantCommand(
            () -> {
              subsystem.setTurretPos(Math.toRadians(Constants.TurretConstants.TURRET_PICKUP_ANGLE));
              subsystem.setPitchPos(Math.toRadians(Constants.TurretConstants.PITCH_PICKUP_ANGLE));
            })
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
        .withName("zeroZeroing")
        .asProxy();
  }

  public Command zeroTurretCommand() {
    if (subsystem == null) return Commands.none();
    return Commands.runOnce(() -> subsystem.zeroTurret(), subsystem)
        .withName("ZeroTurretCommand")
        .asProxy();
  }

  public Command testTurretCommand(double degrees) {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runEnd(() -> subsystem.setTurretPos(Math.toRadians(degrees)), subsystem::stopTurret)
        .withName("TestTurret")
        .asProxy();
  }

  public Command aimTurretCommand() {
    if (subsystem == null) return Commands.none();

    final DoubleSupplier visionSupplier = () -> visionSubsystem.getTurretYaw(0);
    final Supplier<Rotation2d> odometryAngle =
        HowdyMath.getAngleToTargetContinous(
            () -> robotPosition.get().getTranslation(),
            RSM.getAllianceColor().getSpeakerLocation());

    return subsystem.run(
        () -> {
          double visionMeasure = visionSupplier.getAsDouble();
          if (visionMeasure == -1) {
            // No vision for targeting
            subsystem.setTurretPos(
                odometryAngle.get().minus(robotPosition.get().getRotation()).getRadians());
          } else {
            // Using vision
            subsystem.setTurretPos(visionMeasure + subsystem.getTurretPos());
          }
        });
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }
}
