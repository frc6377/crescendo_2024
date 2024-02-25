package frc.robot.subsystems.turretSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.config.TurretZeroConfig;

public class TurretComandFactory {
  final TurretSubsystem subsystem;

  public TurretComandFactory(TurretSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command stowTurret() {
    if (subsystem == null) return new InstantCommand();
    return new InstantCommand(
            () ->
                subsystem.setTurretPos(
                    Math.toRadians(Constants.TurretConstants.TURRET_STOWED_ANGLE)))
        .alongWith(
            new InstantCommand(
                () ->
                    subsystem.setPitchPos(
                        Math.toRadians(Constants.TurretConstants.PITCH_STOWED_ANGLE))))
        .withName("StowTurretCommand");
  }

  public Command pickup() {
    if (subsystem == null) return new InstantCommand();
    return new InstantCommand(
            () ->
                subsystem.setTurretPos(
                    Math.toRadians(Constants.TurretConstants.TURRET_PICKUP_ANGLE)))
        .alongWith(
            new InstantCommand(
                () ->
                    subsystem.setPitchPos(
                        Math.toRadians(Constants.TurretConstants.PITCH_PICKUP_ANGLE))))
        .withName("StowTurretCommand");
  }

  /**
   * A command to set the current turret position as true zero.
   *
   * @return a command that sets the current position as true zero
   */
  public Command zeroZeroing() {
    if (subsystem == null) return new InstantCommand();
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
        subsystem);
  }

  public Command zeroTurretCommand() {
    if (subsystem == null) return new InstantCommand();
    return Commands.runOnce(() -> subsystem.zeroTurret(), subsystem).withName("ZeroTurretCommand");
  }

  public Command moveUpwards() {
    if (subsystem == null) return new InstantCommand();
    return subsystem.run(() -> subsystem.moveUp()).withName("moveShooterUp");
  }

  public Command getAimTurretCommand() {
    if (subsystem == null) return new InstantCommand();
    return subsystem.run(() -> subsystem.aimTurret()).withName("AimTurretCommand");
  }

  public Command idleTurret() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .runEnd(() -> subsystem.holdPosition(), subsystem::stopTurret)
        .withName("idleTurret");
  }

  public Command testTurretCommand(double degrees) {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .runEnd(() -> subsystem.setTurretPos(Math.toRadians(degrees)), subsystem::stopTurret)
        .withName("TestTurret");
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }
}
