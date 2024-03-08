package frc.robot.subsystems.triggerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TriggerConstants;
import java.util.function.BooleanSupplier;

public class TriggerCommandFactory {
  private final TriggerSubsystem subsystem;

  public TriggerCommandFactory(TriggerSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command getGroundLoadCommand(BooleanSupplier tof) {
    return getLoadCommand().until(tof).andThen(getLoadForTime());
  }

  public Command getLoadForTime() {
    if (subsystem == null) return Commands.none();

    return Commands.deadline(new WaitCommand(ShooterConstants.INTAKE_DELAY_SEC), getLoadCommand());
  }

  public Command getLoadCommand() {
    return buildCommand(TriggerConstants.LOAD_PERCENTAGE).withName("getLoadCommand");
  }

  public Command getHoldCommand() {
    return buildCommand(TriggerConstants.HOLD_PERCENTAGE).withName("getHoldCommand");
  }

  public Command getShootCommand() {
    return buildCommand(TriggerConstants.SHOOT_PERCENTAGE).withName("getShootCommand");
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }

  private Command buildCommand(double speed) {
    if (subsystem == null) return Commands.none();
    return new StartEndCommand(
        () -> subsystem.setSpeed(speed), () -> subsystem.setSpeed(0), subsystem);
  }

  public Command getEjectCommand() {
    return buildCommand(-0.25);
  }
}
