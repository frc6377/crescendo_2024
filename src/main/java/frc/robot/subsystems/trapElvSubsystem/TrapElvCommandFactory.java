package frc.robot.subsystems.trapElvSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
    if (subsystem == null) return Commands.none();
    final Command command =
        subsystem.startEnd(() -> subsystem.stowTrapElv(), () -> {}).withName("stowTrapElvCommand");
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

  public Command zeroArm() {
    if (subsystem == null) return Commands.none();
    throw new UnsupportedOperationException("Unimplemented");
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
            () -> {})
        .withName("Score Amp")
        .asProxy();
  }

  public Command wristintakeSource() {
    if (subsystem == null) return Commands.none();
    return intakeSource()
        .until(subsystem.getSourceBreak())
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
        .withName("intakeFromGroundForTime")
        .asProxy();
  }

  public Command intakeFromSourceForTime() {
    if (subsystem == null) return Commands.none();
    return intakeFromSourceForTime(Constants.TrapElvConstants.SOURCE_BEAM_BREAK_DELAY_SEC)
        .withName("intakeFromSourceForTime")
        .asProxy();
  }

  public Command intakeFromSourceForTime(double seconds) {
    if (subsystem == null) return Commands.none();
    return Commands.deadline(new WaitCommand(seconds), intakeSource())
        .withName("intakeFromSourceForTime")
        .asProxy();
  }

  public BooleanSupplier getSourceBreak() {
    // return () -> false;
    return subsystem.getSourceBreak();
  }

  public Command wristShooterRev() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setWristState(TrapElvState.AMP_REV);
            },
            () -> {})
        .withName("Pose AMP");
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }
}
