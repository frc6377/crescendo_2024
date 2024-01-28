package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMaxSim shooterTopMotor;
  private final CANSparkMaxSim shooterBottomMotor;

  private final RelativeEncoder shooterTopEncoder;
  private final RelativeEncoder shooterBottomEncoder;

  public ShooterSubsystem() {
    shooterTopMotor =
        new CANSparkMaxSim(Constants.ShooterConstants.SHOOTER_MOTOR_TOP_ID, MotorType.kBrushless);
    shooterBottomMotor =
        new CANSparkMaxSim(
            Constants.ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID, MotorType.kBrushless);

    shooterTopEncoder = shooterTopMotor.getEncoder();
    shooterBottomEncoder = shooterBottomMotor.getEncoder();

    shooterTopMotor.restoreFactoryDefaults();
    shooterTopMotor.setSmartCurrentLimit(40);
    shooterBottomMotor.restoreFactoryDefaults();
    shooterBottomMotor.setSmartCurrentLimit(40);

    shooterTopMotor.getPIDController().setP(Constants.ShooterConstants.SHOOTER_P);
    shooterTopMotor.getPIDController().setI(Constants.ShooterConstants.SHOOTER_I);
    shooterTopMotor.getPIDController().setD(Constants.ShooterConstants.SHOOTER_D);
    shooterTopMotor.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_FF);
    shooterBottomMotor.getPIDController().setP(Constants.ShooterConstants.SHOOTER_P);
    shooterBottomMotor.getPIDController().setI(Constants.ShooterConstants.SHOOTER_I);
    shooterBottomMotor.getPIDController().setD(Constants.ShooterConstants.SHOOTER_D);
    shooterBottomMotor.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_FF);
  }

  // Fires the shooter. Currently inoperable, must receive distance during runtime without any
  // arguments being passed in!
  public Command shooterFire() {
    return run(
        () -> {
          if (isShooterReady(distance)) {
            setShooterSpeeds(calculateShooterSpeeds(distance));
          }
        });
  }

  // Idle shooter command; for default command purposes
  public Command shooterIdle() {
    return run(
        () -> {
          setShooterSpeeds(Constants.ShooterConstants.SHOOTER_IDLE_SPEEDS);
        });
  }

  // Checks if shooter is ready.
  public boolean isShooterReady(double distance) {
    boolean shooterReady = false;
    double minSpeedTolerance =
        calculateShooterSpeeds(distance)[0]
            * (1 - Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double maxSpeedTolerance =
        calculateShooterSpeeds(distance)[0]
            * (1 + Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    if (minSpeedTolerance < shooterTopMotor.getEncoder().getVelocity()
        & shooterTopMotor.getEncoder().getVelocity() < maxSpeedTolerance) {
      shooterReady = true;
    }

    return shooterReady;
  }

  // Speed in RPM. Top is index 0, bottom is index 1.
  public void setShooterSpeeds(double[] speeds) {
    shooterTopMotor.getPIDController().setReference(speeds[0], CANSparkBase.ControlType.kVelocity);
    shooterBottomMotor
        .getPIDController()
        .setReference(speeds[1], CANSparkBase.ControlType.kVelocity);
  }

  // Top is index 0, bottom is index 1.
  public static double[] calculateShooterSpeeds(double distance) {
    double[] speeds = {0, 0};
    double distanceProportion;

    // If distance below minimum, set speed to minimum.
    if (distance < speakerConfigList[0].getDistance()) {
      speeds[0] = speakerConfigList[0].getSpeedTop();
      speeds[1] = speakerConfigList[0].getSpeedBottom();
    } else {
      // A linear search which determines which points the input distance falls between. May be
      // converted to a binary search if there are many points
      for (int i = 0; i < speakerConfigList.length; i++) {
        // If distance above maximum, set speed to maximum.
        if (i == speakerConfigList.length - 1) {
          speeds[0] = speakerConfigList[i].getSpeedTop();
          speeds[1] = speakerConfigList[i].getSpeedBottom();
          break;
        } else if (distance >= speakerConfigList[i].getDistance()
            && distance < speakerConfigList[i + 1].getDistance()) {
          // Math to linearly interpolate the speed.
          distanceProportion =
              (distance - speakerConfigList[i].getDistance())
                  / (speakerConfigList[i + 1].getDistance() - speakerConfigList[i].getDistance());
          speeds[0] =
              (distanceProportion
                      * (speakerConfigList[i + 1].getSpeedTop()
                          - speakerConfigList[i].getSpeedTop()))
                  + speakerConfigList[i].getSpeedTop();
          speeds[1] =
              (distanceProportion
                      * (speakerConfigList[i + 1].getSpeedBottom()
                          - speakerConfigList[i].getSpeedBottom()))
                  + speakerConfigList[i].getSpeedBottom();
          break;
        }
      }
    }
    return speeds;
  }

  private static class SpeakerConfig {
    private double distance;
    private double speedTop;
    private double speedBottom;

    public SpeakerConfig(double distance, double speedTop, double speedBottom) {
      this.distance = distance;
      this.speedTop = speedTop;
      this.speedBottom = speedBottom;
    }

    public double getDistance() {
      return distance;
    }

    public double getSpeedTop() {
      return speedTop;
    }

    public double getSpeedBottom() {
      return speedBottom;
    }
  }

  private static SpeakerConfig[] speakerConfigList = {
    new SpeakerConfig(0, 450, 250),
    new SpeakerConfig(40, 550, 350),
    new SpeakerConfig(195, 750, 500),
    new SpeakerConfig(290, 1000, 700)
  };
}
