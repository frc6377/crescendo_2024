package frc.robot.subsystems.triggerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TriggerConstants;
import frc.robot.utilities.TunableNumber;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class TriggerCommandFactory {
  private final TriggerSubsystem subsystem;
  private final TunableNumber shootPercentage;

  public TriggerCommandFactory(TriggerSubsystem subsystem) {
    this.subsystem = subsystem;
    shootPercentage =
        new TunableNumber("Fire  percent", TriggerConstants.SHOOT_PERCENTAGE, subsystem);
  }

  public Command getGroundLoadCommand(BooleanSupplier tof) {
    return getLoadCommand().until(tof).andThen(getLoadForTime()).withName("getGroundLoadCommand");
  }

  public Command getLoadForTime() {
    if (subsystem == null) return Commands.none();

    return Commands.deadline(new WaitCommand(ShooterConstants.INTAKE_DELAY_SEC), getLoadCommand())
        .withName("getLoadForTime");
  }

  public Command getLoadCommand() {
    return buildCommand(TriggerConstants.LOAD_PERCENTAGE).withName("getLoadCommand").asProxy();
  }

  public Command getHoldCommand() {
    final Command command =
        buildCommand(TriggerConstants.HOLD_PERCENTAGE).withName("getHoldCommand").asProxy();
    return command;
  }

  public Command getShootCommand() {
    return buildCommand(shootPercentage.getAsDouble()).withName("getShootCommand").asProxy();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(
        Commands.parallel(subsystem.runOnce(() -> {}), defaultCommand)
            .withName(defaultCommand.getName()));
  }

  private Command buildCommand(double speed) {
    if (subsystem == null) return Commands.none();
    return new StartEndCommand(
            () -> subsystem.setSpeed(speed), () -> subsystem.setSpeed(0), subsystem)
        .withName("buildCommand");
  }

  public Command getEjectCommand() {
    return buildCommand(TriggerConstants.EJECT_PERCENT).asProxy();
  }

  public Command[] getCommands() {
    ArrayList<Command> cmds = new ArrayList<Command>();
    cmds.add(getGroundLoadCommand(() -> true));
    cmds.add(getLoadForTime());
    cmds.add(getLoadCommand());
    cmds.add(getHoldCommand());
    cmds.add(getShootCommand());
    cmds.add(getEjectCommand());
    return cmds.toArray(new Command[cmds.size()]);
  }
}
