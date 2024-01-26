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
          //shooterIdle();
        });
  }

  // TODO: Fold into shooterFire
  public Command readyShooter(final double distance) {
    // TargetType target =  ||| Take in target type from vision / elsewhere
    return runOnce(
        () -> {
          setShooterSpeed(calculateShooterSpeed(distance)); // Take in distance from vision
        });
  }

  public Command shooterIdle() {
    return run(
        () -> {
          setShooterSpeed(Constants.ShooterConstants.SHOOTER_MINIMUM_SPEED);
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
  // Distance in inches.
  public double calculateShooterSpeed(double distance) {
    double speed = 0;

    // Distance-speed pairs, in inches and RPM respectively.
    double[] pointOne = {0, 0};
    double[] pointTwo = {0, 0};

    // A linear search which determines which points the input distance falls between. May be converted to a binary search if there are many points
    for (int i = 0; i <= Constants.ShooterConstants.SpeakerRanges.SpeakerRanges.length; i++) {
      if (i == Constants.ShooterConstants.SpeakerRanges.SpeakerRanges.length) {
        pointOne = Constants.ShooterConstants.SpeakerRanges.SpeakerRanges[i];
        pointTwo = Constants.ShooterConstants.SpeakerRanges.SpeakerRanges[i];
      }
      else if (speed > Constants.ShooterConstants.SpeakerRanges.SpeakerRanges[i][1]) {
        pointOne = Constants.ShooterConstants.SpeakerRanges.SpeakerRanges[i];
        pointTwo = Constants.ShooterConstants.SpeakerRanges.SpeakerRanges[i+1];
      }
    }

    double distanceProportion = (distance - pointOne[0]) / (pointTwo[0] - pointOne[0]);
    speed = (distanceProportion * (pointTwo[1] - pointOne[1])) + pointOne[1];

    return speed;
  }
}
