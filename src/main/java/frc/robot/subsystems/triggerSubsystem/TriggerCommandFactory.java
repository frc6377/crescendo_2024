package frc.robot.subsystems.triggerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.TriggerConstants;

public class TriggerCommandFactory {
  private final TriggerSubsystem subsystem;

  public TriggerCommandFactory(TriggerSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command getLoadCommand() {
    return buildCommand(TriggerConstants.LOAD_PERCENTAGE).withName("getLoadCommand").asProxy();
  }

  public Command getHoldCommand() {
    return buildCommand(TriggerConstants.HOLD_PERCENTAGE).withName("getHoldCommand").asProxy();
  }

  public Command getShootCommand() {
    return buildCommand(TriggerConstants.SHOOT_PERCENTAGE).withName("getShootCommand").asProxy();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }

  private Command buildCommand(double speed) {
    if (subsystem == null) return new InstantCommand().withName("buildCommand").asProxy();
    return new StartEndCommand(
            () -> subsystem.setSpeed(speed), () -> subsystem.setSpeed(0), subsystem)
        .withName("buildCommand")
        .asProxy();
  }
}
