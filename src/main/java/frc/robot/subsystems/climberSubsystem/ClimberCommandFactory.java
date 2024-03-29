package frc.robot.subsystems.climberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;

/** Expected climb sequece is: 1. Raise 2. Clip 3. Climb */
public class ClimberCommandFactory {
  private final ClimberSubsystem subsystem;
  private final Runnable noop = () -> {};

  public ClimberCommandFactory(ClimberSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command initalRaise() {
    if (subsystem == null) return Commands.none();
    return new InstantCommand(() -> subsystem.setMaximum())
        .andThen(new WaitCommand(ClimberConstants.BREAK_STATIC_TIME))
        .withName("initalRaise")
        .asProxy();
  }

  public Command advanceClimbStage() {
    if (subsystem == null) return Commands.none();
    return new InstantCommand(() -> subsystem.nextStage())
        .andThen(new WaitCommand(ClimberConstants.BREAK_STATIC_TIME))
        .withName("advanceClimbStage")
        .asProxy();
  }
}
