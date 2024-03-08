package frc.robot.subsystems.triggerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    final Command command =
        buildCommand(TriggerConstants.HOLD_PERCENTAGE).withName("getHoldCommand");
    return command;
  }

  public Command getShootCommand() {
    return buildCommand(TriggerConstants.SHOOT_PERCENTAGE).withName("getShootCommand").asProxy();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }

  private Command buildCommand(double speed) {
    if (subsystem == null) return Commands.none();
    return new StartEndCommand(
            () -> subsystem.setSpeed(speed), () -> subsystem.setSpeed(0), subsystem)
        .withName("buildCommand");
  }
}
