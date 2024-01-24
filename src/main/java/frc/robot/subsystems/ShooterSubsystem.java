package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotor;
  boolean readyToFire = true; // Take in readiness state from ShooterTrigger?

  public ShooterSubsystem() {
    shooterMotor =
        new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setSmartCurrentLimit(40);
  }

  public Command shooterFire() {
    return startEnd(
        () -> {
          if (readyToFire) {
            readyShooter();
          }
        },
        () -> {
          setShooterSpeed(0);
        });
  }

  public void readyShooter() {
    double speed = 0;
    double distance = 0; // Take in distance from vision
    // TargetType target =  ||| Take in target type from vision / elsewhere

    speed = calculateShooterSpeed(distance); // Include target
    setShooterSpeed(speed);
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  // Parameter: TargetType target ???
  public double calculateShooterSpeed(double distance) {
    double speed = distance; // Placeholder
    return speed;
  }
}
