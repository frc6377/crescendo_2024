package frc.robot.subsystems.climberSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/** Expected climb sequece is: 1. Raise 2. Clip 3. Climb */
public class ClimberCommandFactory {
  private final ClimberSubsystem subsystem;
  private final Runnable noop = () -> {};

  public ClimberCommandFactory(ClimberSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command raise() {
    if (subsystem == null) return Commands.none();
    Timer minTime = new Timer();
    return initalRaise()
        .andThen(
            new FunctionalCommand(
                () -> {
                  subsystem.setOutputLimits(1, 0);
                  subsystem.applyCurrentDemand(ClimberConstants.RAISE_CURRENT);
                  minTime.start();
                },
                noop,
                (interupt) -> {
                  subsystem.setOutputLimits(0, -1);
                  subsystem.applyPercent(0);
                },
                () -> {
                  boolean vel = subsystem.getVelocity().isZero(0.5);
                  return vel && minTime.hasElapsed(ClimberConstants.MIN_RAISE_TIME_SEC);
                },
                subsystem))
        .withName("raise")
        .asProxy();
  }

  public Command initalRaise() {
    if (subsystem == null) return Commands.none();
    return new InstantCommand(() -> subsystem.applyPercent(ClimberConstants.INITAL_RAISE_PERCENT))
        .andThen(new WaitCommand(ClimberConstants.BREAK_STATIC_TIME))
        .withName("initalRaise")
        .asProxy();
  }

  public Command clip() {
    if (subsystem == null) return Commands.none();
    return breakStatic()
        .andThen(subsystem.run(() -> subsystem.applyCurrentDemand(ClimberConstants.CLIP_CURRENT)))
        .withName("clip")
        .asProxy();
  }

  public Command breakStatic() {
    if (subsystem == null) return Commands.none();
    return new InstantCommand(() -> subsystem.applyPercent(ClimberConstants.BREAK_STATIC_PERCENT))
        .andThen(new WaitCommand(ClimberConstants.BREAK_STATIC_TIME))
        .withName("breakStatic")
        .asProxy();
  }

  public Command climb() {
    if (subsystem == null) return Commands.none();

    Runnable init =
        () -> {
          subsystem.requestPosition(ClimberConstants.CLIMB_POSITION);
        };
    Runnable exec = () -> {};
    Consumer<Boolean> end = (interupt) -> {};
    BooleanSupplier isFinished =
        () -> subsystem.getPosition().isLessThen(ClimberConstants.CLIMB_POSITION);

    return new FunctionalCommand(init, exec, end, isFinished, subsystem)
        .withName("climb")
        .asProxy();
  }

  public Command[] getCommands() {
    ArrayList<Command> cmds = new ArrayList<Command>();
    cmds.add(raise());
    cmds.add(clip());
    cmds.add(climb());
    cmds.add(breakStatic());
    cmds.add(initalRaise());
    return cmds.toArray(new Command[cmds.size()]);
  }
}
