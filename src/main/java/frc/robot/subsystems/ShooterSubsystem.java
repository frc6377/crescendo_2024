package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax shooterMotor;
  boolean readyToFire = true; // Take in readiness state from ShooterTrigger?

  public ShooterSubsystem() {
    shooterMotor =
        new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setSmartCurrentLimit(40);
  }

  public Command shooterFire(final double distance) {
    return startEnd(
        () -> {
          if (readyToFire) {
            readyShooter(distance);
          }
        },
        () -> {
          setShooterSpeed(0);
        });
  }

  // Fold into shooterFire if checking shooter readiness / piece handling isn't necessary
  public Command readyShooter(final double distance) {
    // TargetType target =  ||| Take in target type from vision / elsewhere
    return runOnce(
        () -> {
          setShooterSpeed(calculateShooterSpeed(distance)); // Take in distance from vision
        });
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
