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
    shooterMotor.getPIDController().setP(Constants.ShooterConstants.SHOOTER_P);
    shooterMotor.getPIDController().setI(Constants.ShooterConstants.SHOOTER_I);
    shooterMotor.getPIDController().setD(Constants.ShooterConstants.SHOOTER_D);
    shooterMotor.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_FF);
  }

  // Fires the shooter.
  public Command shooterFire(double distance) {
    return startEnd(
        () -> {
          if (isShooterReady(distance)) {
            setShooterSpeed(calculateShooterSpeed(distance));
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
          setShooterSpeed(Constants.ShooterConstants.SHOOTER_IDLE_SPEED);
        });
  }

  // Checks if shooter is ready.
  public boolean isShooterReady(double distance) {
    boolean shooterReady = false;
    double minSpeedTolerance =
        calculateShooterSpeed(distance) * (1 - Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double maxSpeedTolerance =
        calculateShooterSpeed(distance) * (1 + Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);

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

  public static double calculateShooterSpeed(double distance) {
    double speed = 0;
    double distanceProportion;
    
    // If distance below minimum, set speed to minimum.
    if (distance < speakerConfigList[0].getDistance()) {
      speed = speakerConfigList[0].getSpeed();
    }
    else {
      // A linear search which determines which points the input distance falls between. May be
      // converted to a binary search if there are many points
      for (int i = 0; i < speakerConfigList.length; i++) {
        // If distance above maximum, set speed to maximum.
        if (i == speakerConfigList.length - 1) {
          speed = speakerConfigList[i].getSpeed();
          break;
        } else if (distance >= speakerConfigList[i].getDistance()) {
          // Math to linearly interpolate the speed.
          distanceProportion =
              (distance - speakerConfigList[i].getDistance()) / (speakerConfigList[i + 1].getDistance() - speakerConfigList[i].getDistance());
          speed =
              (distanceProportion * (speakerConfigList[i + 1].getSpeed() - speakerConfigList[i].getSpeed()))
                  + speakerConfigList[i].getSpeed();
          break;
        }
      }
    }
    return speed;
  }

  private static class SpeakerConfig {
    private double distance;
    private double speed;

    public SpeakerConfig(double distance, double speed) {
      this.distance = distance;
      this.speed = speed;
    }

    public double getDistance() {
      return distance;
    }

    public double getSpeed() {
      return speed;
    }
  }

  private static SpeakerConfig[] speakerConfigList = {
      new SpeakerConfig(0, 450),
      new SpeakerConfig(40, 550),
      new SpeakerConfig(195, 750),
      new SpeakerConfig(290, 1000)
  };
}
