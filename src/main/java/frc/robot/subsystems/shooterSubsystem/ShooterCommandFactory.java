package frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem.SpeakerConfig;

public class ShooterCommandFactory {
  private final ShooterSubsystem subsystem;
  private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterSubsystem");
  private GenericEntry targetRPM = shooterTab.add("Target RPM", 0).getEntry();
  private GenericEntry rightTargetRPM = shooterTab.add("right RPM", 0).getEntry();

  public ShooterCommandFactory(ShooterSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command intakeSource() {
    if (subsystem == null) return new InstantCommand();
    return subsystem.startEnd(
        () -> {
          subsystem.setShooterSpeeds(ShooterConstants.SHOOTER_SOURCE_INTAKE);
        },
        () -> {});
  }

  public Command intakeSourceForTime() {
    if (subsystem == null) return new InstantCommand();
    return Commands.deadline(new WaitCommand(ShooterConstants.INTAKE_DELAY_SEC), intakeSource());
  }

  public Command intakeSpeakerSource() {
    if (subsystem == null) return new InstantCommand();
    return intakeSource().until(subsystem.getBeamBreak()).andThen(intakeSourceForTime());
  }

  // Spins up the shooter, and requests feeding it when the rollers are within parameters.
  // Receives distance-to-target from Limelight, or other sensor.
  // Required to be called repeatedly; consider pub-sub for LimelightGetDistance() or equivalent
  // method to save a method call
  public Command revShooter() {
    if (subsystem == null) return new InstantCommand();
    return Commands.startEnd(
        () -> {
          subsystem.setShooterSpeeds(
              new SpeakerConfig(-1, targetRPM.getDouble(0), rightTargetRPM.getDouble(0)));
          System.out.println("Work I dare thee");
        },
        () -> {},
        subsystem);
  }

  // Idle shooter command; for default command purposes
  public Command shooterIdle() {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .run(
            () -> {
              subsystem.stop();
            })
        .withName("Idle Shooter command");
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }
}
