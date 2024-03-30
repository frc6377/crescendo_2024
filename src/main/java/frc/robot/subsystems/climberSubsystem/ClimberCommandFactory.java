package frc.robot.subsystems.climberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import java.util.ArrayList;

/** Expected climb sequece is: 1. Raise 2. Clip 3. Climb */
public class ClimberCommandFactory {
  private final ClimberSubsystem subsystem;

  public ClimberCommandFactory(ClimberSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command initalRaise() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runOnce(() -> subsystem.setMaximum())
        .andThen(new WaitCommand(ClimberConstants.BREAK_STATIC_TIME))
        .withName("initalRaise")
        .asProxy();
  }

  public Command advanceClimbStage() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runOnce(() -> subsystem.nextStage())
        .andThen(new WaitCommand(ClimberConstants.BREAK_STATIC_TIME))
        .withName("advanceClimbStage")
        .asProxy();
  }

  public Command[] getCommands() {
    ArrayList<Command> cmds = new ArrayList<Command>();
    cmds.add(initalRaise());
    cmds.add(advanceClimbStage());
    return cmds.toArray(new Command[cmds.size()]);
  }
}
