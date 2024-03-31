package frc.robot.subsystems.climberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climberSubsystem.ClimberSubsystem.ClimberState;
import java.util.ArrayList;

/** Expected climb sequece is: 1. Raise 2. Clip 3. Climb */
public class ClimberCommandFactory {
  private final ClimberSubsystem subsystem;

  public ClimberCommandFactory(ClimberSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command raise() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runOnce(() -> subsystem.setState(ClimberState.INITIAL_RAISE))
        .withName("Raise Climber")
        .asProxy();
  }

  public Command advanceState() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runOnce(() -> subsystem.advanceState())
        .withName("Advance Climber Stage")
        .asProxy();
  }

  public Command[] getCommands() {
    ArrayList<Command> cmds = new ArrayList<Command>();
    cmds.add(raise());
    cmds.add(advanceState());
    return cmds.toArray(new Command[cmds.size()]);
  }
}
