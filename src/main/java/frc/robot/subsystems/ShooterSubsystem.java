package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax shooterMotor;

  public ShooterSubsystem() {
    shooterMotor =
        new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setSmartCurrentLimit(40);

    // Placeholder values
    shooterMotor.getPIDController().setP(0);
    shooterMotor.getPIDController().setI(0);
    shooterMotor.getPIDController().setD(0);
    shooterMotor.getPIDController().setFF(0);
  }

  public Command shooterFire(final double distance) {
    return startEnd(
        () -> {
          if (isShooterReady(distance)) {
            readyShooter(distance);
          }
        },
        () -> {
          setShooterSpeed(Constants.ShooterConstants.SHOOTER_MINIMUM_SPEED);
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

  public boolean isShooterReady(double distance) {
    boolean shooterReady = false;
    double minSpeedTolerance = calculateShooterSpeed(distance)*(1-Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double maxSpeedTolerance = calculateShooterSpeed(distance)*(1+Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    if (minSpeedTolerance < shooterMotor.getEncoder().getVelocity() & shooterMotor.getEncoder().getVelocity() < maxSpeedTolerance) {
      shooterReady = true;
    }

    return shooterReady;
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  // Parameter: TargetType target ???
  public double calculateShooterSpeed(double distance) {
    double speed = distance; // Placeholder
    return speed;
  }
}
