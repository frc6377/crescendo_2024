package frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem.SpeakerConfig;
import frc.robot.utilities.TunableNumber;
import java.util.ArrayList;

public class ShooterCommandFactory {
  private final ShooterSubsystem subsystem;
  private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterSubsystem");
  private TunableNumber leftTargetRPM;
  private TunableNumber rightTargetRPM;

  public ShooterCommandFactory(ShooterSubsystem subsystem) {
    this.subsystem = subsystem;
    if (subsystem != null) {
      leftTargetRPM =
          new TunableNumber("Left RPM", ShooterConstants.SHOOTER_LEFT_TARGET_RPM, subsystem);
      rightTargetRPM =
          new TunableNumber("Right RPM", ShooterConstants.SHOOTER_RIGHT_TARGET_RPM, subsystem);
    }
  }

  public Command intakeSource() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setShooterSpeeds(ShooterConstants.SHOOTER_SOURCE_INTAKE);
            },
            () -> {})
        .withName("intakeSource")
        .asProxy();
  }

  public Command intakeSourceForTime() {
    if (subsystem == null) return Commands.none();
    return Commands.deadline(new WaitCommand(ShooterConstants.INTAKE_DELAY_SEC), intakeSource())
        .withName("intakeSourceForTime")
        .asProxy();
  }

  public Command intakeSpeakerSource() {
    if (subsystem == null) return Commands.none();
    return intakeSource()
        .until(subsystem.getBeamBreak())
        .andThen(intakeSourceForTime())
        .withName("intakeSpeakerSource")
        .asProxy();
  }

  // Spins up the shooter, and requests feeding it when the rollers are within parameters.
  // Receives distance-to-target from Limelight, or other sensor.
  // Required to be called repeatedly; consider pub-sub for LimelightGetDistance() or equivalent
  // method to save a method call
  public Command revShooter() {
    if (subsystem == null) return Commands.none();
    return new FunctionalCommand(
            () -> {
              subsystem.setShooterSpeeds(
                  new SpeakerConfig(-1, leftTargetRPM.get(), rightTargetRPM.get()));
            },
            () -> {},
            (a) -> {},
            () -> false,
            subsystem)
        .withName("revShooter")
        .asProxy();
  }

  // Idle shooter command; for default command purposes
  public Command shooterIdle() {
    if (subsystem == null) return Commands.none();
    final Command command =
        subsystem
            .run(
                () -> {
                  subsystem.stop();
                })
            .withName("Idle Shooter command")
            .asProxy();
    return command;
  }

  public Command outtake() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(() -> subsystem.requestPercent(-1), subsystem::stop)
        .withName("outtake")
        .asProxy();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(Commands.sequence(subsystem.runOnce(() -> {}), defaultCommand));
  }

  public boolean isShooterReady() {
    if (subsystem == null) return true;
    return subsystem.isShooterReady();
  }

  public Trigger getBeamBreak() {
    if (subsystem == null) return new Trigger(() -> false);
    return subsystem.getBeamBreak();
  }

  public Command[] getCommands() {
    ArrayList<Command> cmds = new ArrayList<Command>();
    cmds.add(this.intakeSource());
    cmds.add(this.intakeSourceForTime());
    cmds.add(this.intakeSpeakerSource());
    cmds.add(this.outtake());
    cmds.add(this.revShooter());
    cmds.add(this.shooterIdle());
    return cmds.toArray(new Command[cmds.size()]);
  }
}
