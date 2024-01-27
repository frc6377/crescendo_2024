package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
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

  // Fires the shooter.
  public Command shooterFire(double distance) {
    return startEnd(
        () -> {
          if (isShooterReady(distance)) {
            setShooterSpeed(SpeakerRanges.getSpeed(distance));
          }
        },
        () -> {
          // shooterIdle();
        });
  }

  // Idle shooter command; for default command purposes
  public Command shooterIdle() {
    return run(
        () -> {
          setShooterSpeed(Constants.ShooterConstants.SHOOTER_MINIMUM_SPEED);
        });
  }

  // Checks if shooter is ready.
  public boolean isShooterReady(double distance) {
    boolean shooterReady = false;
    double minSpeedTolerance =
        SpeakerRanges.getSpeed(distance) * (1 - Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double maxSpeedTolerance =
        SpeakerRanges.getSpeed(distance) * (1 + Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    if (minSpeedTolerance < shooterMotor.getEncoder().getVelocity()
        & shooterMotor.getEncoder().getVelocity() < maxSpeedTolerance) {
      shooterReady = true;
    }

    return shooterReady;
  }

  // Speed in RPM.
  public void setShooterSpeed(double speed) {
    shooterMotor.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  private class SpeakerRanges {
    private static double speed;

    // Placeholder; in inches and RPM respectively.
    private static double[][] SpeakerRanges = {
      {0, 450},
      {40, 550},
      {195, 750},
      {290, 1000}
    };

    public static double getSpeed(double distance) {
      double distanceProportion;
      // A linear search which determines which points the input distance falls between. May be
      // converted to a binary search if there are many points
      for (int i = 0; i <= SpeakerRanges.length; i++) {
        if (i == SpeakerRanges.length) {
          // Maximum possible speed.
          speed = SpeakerRanges[i][1];
          break;
        } else if (distance > SpeakerRanges[i][0]) {
          // Math to linearly interpolate the speed.
          distanceProportion =
              (distance - SpeakerRanges[i][0]) / (SpeakerRanges[i + 1][0] - SpeakerRanges[i][0]);
          speed =
              (distanceProportion * (SpeakerRanges[i + 1][1] - SpeakerRanges[i][1]))
                  + SpeakerRanges[i][1];
          break;
        }
      }
      return speed;
    }
  }
}
