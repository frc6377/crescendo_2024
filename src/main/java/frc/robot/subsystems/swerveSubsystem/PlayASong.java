package frc.robot.subsystems.swerveSubsystem;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class PlayASong extends Command {
  private Orchestra orchestra;
  private String song = "";

  public PlayASong(SwerveSubsystem subsystem) {
    this(subsystem, "");
  }

  public PlayASong(SwerveSubsystem subsystem, String song) {
    this.song = song;
    orchestra = new Orchestra();
    grabMotors(subsystem, orchestra);
  }

  private static void grabMotors(SwerveSubsystem subsystem, Orchestra orchestra) {
    for (int i = 0; subsystem.getModule(i) != null; i++) {
      SwerveModule module = subsystem.getModule(i);
      orchestra.addInstrument(module.getDriveMotor());
      orchestra.addInstrument(module.getSteerMotor());
    }
  }

  public void setSong(String song) {
    this.song = song;
  }

  @Override
  public void initialize() {
    if (song == "") {
      DriverStation.reportError("Drive Train Misuse Requires a song", true);
    }
    orchestra.loadMusic(song);
    this.ignoringDisable(true);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interupted) {
    orchestra.stop();
  }

  @Override
  public boolean isFinished() {
    return orchestra.isPlaying();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
