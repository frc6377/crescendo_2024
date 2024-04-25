package frc.robot.subsystems.trapElvSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.TrapElvConstants;
import frc.robot.subsystems.trapElvSubsystem.TrapElvSubsystem.TrapElvState;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import javax.annotation.Nullable;

public class TrapElvCommandFactory {
  @Nullable private final TrapElvSubsystem subsystem;

  public TrapElvCommandFactory(@Nullable TrapElvSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command stowTrapElvCommand() {
    if (subsystem == null) return Commands.none();
    final Command command =
        subsystem
            .startEnd(() -> subsystem.stowTrapElv(), () -> {})
            .withName("stowTrapElvCommand")
            .asProxy();
    return command;
  }

  public Command setWristSource() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .run(
            () -> {
              subsystem.setWristState(TrapElvState.FROM_SOURCE);
            })
        .withName("setWristSource")
        .asProxy();
  }

  public Command setWristAMP() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .run(
            () -> {
              subsystem.setWristState(TrapElvState.AMP_SCORE);
            })
        .withName("setWristAMP")
        .asProxy();
  }

  public Command setWristStowed() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .run(
            () -> {
              subsystem.setWristState(TrapElvState.STOWED);
            })
        .withName("setWristStowed")
        .asProxy();
  }

  public Command rollerIntakeCommand() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runEnd(
            () -> {
              subsystem.setRoller(TrapElvConstants.ROLLER_SPEED);
            },
            () -> {
              subsystem.setRoller(0);
            })
        .withName("rollerIntakeCommand")
        .asProxy();
  }

  public Command rollerOutakeCommand() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .runEnd(
            () -> {
              subsystem.setRoller(TrapElvConstants.ROLLER_REVERSE_SPEED);
            },
            () -> {
              subsystem.setRoller(0);
            })
        .withName("rollerOutakeCommand")
        .asProxy();
  }

  public Command stopRoller() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .run(
            () -> {
              subsystem.setRoller(0);
            })
        .withName("Stop Roller")
        .asProxy();
  }

  public Command intakeSource() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setWristState(TrapElvState.FROM_SOURCE);
              subsystem.setRoller(TrapElvConstants.ROLLER_SPEED);
            },
            () -> {})
        .withName("Intake From Source")
        .asProxy();
  }

  public Command intakeGround() {
    if (subsystem == null) return new StartEndCommand(() -> {}, () -> {});
    return subsystem
        .startEnd(
            () -> {
              subsystem.setWristState(TrapElvState.STOWED);
              subsystem.setRoller(TrapElvConstants.ROLLER_SPEED);
            },
            () -> {
              subsystem.stowTrapElv();
            })
        .withName("Intake from Ground")
        .asProxy();
  }

  public Command positionAMP() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setWristState(TrapElvState.AMP_SCORE);
            },
            () -> {})
        .withName("Pose AMP")
        .asProxy();
  }

  public Command scoreAMP() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setRoller(TrapElvConstants.ROLLER_SPEED);
            },
            () -> {
              subsystem.setRoller(0);
            })
        .withName("Score Amp")
        .asProxy();
  }

  public Command wristintakeSource() {
    if (subsystem == null) return Commands.none();
    return intakeSource()
        .until(subsystem.getWristBeamBreak())
        .andThen(Commands.print("0.25s left").andThen(intakeFromSourceForTime()))
        .withName("wristIntakeSource")
        .asProxy();
  }

  public Command intakeFromGroundForTime() {
    if (subsystem == null) return Commands.none();
    return intakeFromGroundForTime(Constants.TrapElvConstants.INTAKE_BEAM_BREAK_DELAY_SEC)
        .withName("intakeFromGroundForTime")
        .asProxy();
  }

  public Command intakeFromGroundForTime(double seconds) {
    if (subsystem == null) return Commands.none();
    return Commands.deadline(new WaitCommand(seconds), intakeGround())
        .withName("intakeFromGroundForTimeDefault")
        .asProxy();
  }

  public Command intakeFromSourceForTime() {
    if (subsystem == null) return Commands.none();
    return intakeFromSourceForTime(Constants.TrapElvConstants.SOURCE_BEAM_BREAK_DELAY_SEC)
        .withName("intakeFromSourceForTimeDefault")
        .asProxy();
  }

  public Command intakeFromSourceForTime(double seconds) {
    if (subsystem == null) return Commands.none();
    return Commands.deadline(new WaitCommand(seconds), intakeSource())
        .withName("intakeFromSourceForTime")
        .asProxy();
  }

  public BooleanSupplier getSourceBreak() {
    if (subsystem == null) return () -> true;
    return subsystem.getWristBeamBreak();
  }

  public Command shooterMoving() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setWristState(TrapElvState.AMP_REV);
            },
            () -> {})
        .withName("Shooter Moving")
        .asProxy();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(
        Commands.parallel(subsystem.runOnce(() -> {}), defaultCommand)
            .withName(defaultCommand.getName()));
  }

  public Command[] getCommands() {
    ArrayList<Command> cmds = new ArrayList<Command>();
    cmds.add(this.intakeSource());
    cmds.add(this.intakeGround());
    cmds.add(this.positionAMP());
    cmds.add(this.rollerIntakeCommand());
    cmds.add(this.rollerOutakeCommand());
    cmds.add(this.stopRoller());
    cmds.add(this.scoreAMP());
    cmds.add(this.setWristAMP());
    cmds.add(this.setWristSource());
    cmds.add(this.setWristStowed());
    cmds.add(this.stowTrapElvCommand());
    cmds.add(this.wristintakeSource());
    cmds.add(this.intakeFromGroundForTime());
    cmds.add(this.intakeFromSourceForTime());
    cmds.add(this.intakeFromGroundForTime(0.1));
    cmds.add(this.intakeFromSourceForTime(0.1));
    cmds.add(this.shooterMoving());
    return cmds.toArray(new Command[cmds.size()]);
  }
}
