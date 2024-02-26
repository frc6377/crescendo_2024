package frc.robot.subsystems.trapElvSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.TrapElvConstants;
import frc.robot.subsystems.trapElvSubsystem.TrapElvSubsystem.TrapElvState;
import java.util.function.BooleanSupplier;

public class TrapElvCommandFactory {
  private final TrapElvSubsystem subsystem;

  public TrapElvCommandFactory(TrapElvSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command stowTrapElvCommand() {
    if (subsystem == null) return new InstantCommand();
    return subsystem.startEnd(() -> subsystem.stowTrapElv(), () -> {});
  }

  public Command setWristSource() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .run(
            () -> {
              subsystem.setWristState(TrapElvState.FROM_SOURCE);
            })
        .withName("setWristSource");
  }

  public Command setWristAMP() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .run(
            () -> {
              subsystem.setWristState(TrapElvState.AMP_SCORE);
            })
        .withName("setWristAMP");
  }

  public Command setWristStowed() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .run(
            () -> {
              subsystem.setWristState(TrapElvState.STOWED);
            })
        .withName("setWristStowed");
  }

  public Command rollerIntakeCommand() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .runEnd(
            () -> {
              subsystem.setRoller(TrapElvConstants.ROLLER_SPEED);
            },
            () -> {
              subsystem.setRoller(0);
            })
        .withName("rollerIntakeCommand");
  }

  public Command rollerOutakeCommand() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .runEnd(
            () -> {
              subsystem.setRoller(TrapElvConstants.ROLLER_REVERSE_SPEED);
            },
            () -> {
              subsystem.setRoller(0);
            })
        .withName("rollerOutakeCommand");
  }

  public Command zeroArm() {
    if (subsystem == null) return new InstantCommand();
    throw new UnsupportedOperationException("Unimplemented");
  }

  public Command stopRoller() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .run(
            () -> {
              subsystem.setRoller(0);
            })
        .withName("Stop Roller");
  }

  public Command intakeSource() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setTrapArm(TrapElvState.FROM_SOURCE);
              subsystem.setRoller(TrapElvConstants.ROLLER_SPEED);
            },
            () -> {})
        .withName("Intake From Source");
  }

  public Command intakeGround() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setRoller(TrapElvConstants.ROLLER_SPEED);
            },
            () -> {
              subsystem.stowTrapElv();
            })
        .withName("Intake from Ground");
  }

  public Command positionAMP() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setTrapArm(TrapElvState.AMP_SCORE);
            },
            () -> {})
        .withName("Score Amp");
  }

  public Command scoreAMP() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setRoller(-TrapElvConstants.ROLLER_SPEED);
            },
            () -> {})
        .withName("Score Amp");
  }

  public Command wristintakeSource() {
    if (subsystem == null) return new InstantCommand();
    return intakeSource().until(subsystem.getSourceBreak()).andThen(intakeFromSourceForTime());
  }

  public Command intakeFromGroundForTime() {
    if (subsystem == null) return new InstantCommand();
    return intakeFromGroundForTime(Constants.TrapElvConstants.INTAKE_BEAM_BREAK_DELAY_SEC);
  }

  public Command intakeFromGroundForTime(double seconds) {
    if (subsystem == null) return new InstantCommand();
    return Commands.deadline(intakeGround(), new WaitCommand(seconds));
  }

  public Command intakeFromSourceForTime() {
    if (subsystem == null) return new InstantCommand();
    return intakeFromSourceForTime(Constants.TrapElvConstants.INTAKE_BEAM_BREAK_DELAY_SEC);
  }

  public Command intakeFromSourceForTime(double seconds) {
    if (subsystem == null) return new InstantCommand();
    return Commands.deadline(intakeSource(), new WaitCommand(seconds));
  }

  public BooleanSupplier getSourceBreak() {
    if (subsystem == null) return () -> false;
    return subsystem.getSourceBreak();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }
}
