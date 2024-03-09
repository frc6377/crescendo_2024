package frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem.SpeakerConfig;

public class ShooterCommandFactory {
  private final ShooterSubsystem subsystem;
  private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterSubsystem");
  private GenericEntry targetRPM = shooterTab.add("Target RPM", 2750).getEntry();
  private GenericEntry rightTargetRPM = shooterTab.add("right RPM", 2350).getEntry();

  public ShooterCommandFactory(ShooterSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command intakeSource() {
    if (subsystem == null) return Commands.none();
    return subsystem.startEnd(
        () -> {
          subsystem.setShooterSpeeds(ShooterConstants.SHOOTER_SOURCE_INTAKE);
        },
        () -> {});
  }

  public Command intakeSourceForTime() {
    if (subsystem == null) return Commands.none();
    return Commands.deadline(new WaitCommand(ShooterConstants.INTAKE_DELAY_SEC), intakeSource());
  }

  public Command intakeSpeakerSource() {
    if (subsystem == null) return Commands.none();
    return intakeSource().until(subsystem.getBeamBreak()).andThen(intakeSourceForTime());
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
              new SpeakerConfig(-1, targetRPM.getDouble(4000), rightTargetRPM.getDouble(4000)));
        },
        () -> {},
        (a) -> {},
        () -> false,
        subsystem);
  }

  // Idle shooter command; for default command purposes
  public Command shooterIdle() {
    if (subsystem == null) return Commands.none();
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

  public Command outtake() {
    if (subsystem == null) return Commands.none();
    return subsystem.startEnd(() -> subsystem.requestPercent(-1), subsystem::stop);
  }

  public Trigger isShooterReadyTrigger() {
    if (subsystem == null) return new Trigger(() -> false);
    return new Trigger(subsystem::isShooterReady);
  }
}
