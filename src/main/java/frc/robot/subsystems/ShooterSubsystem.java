package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax shooterTopMotor;
  private final CANSparkMax shooterBottomMotor;

  public ShooterSubsystem() {
    shooterTopMotor =
        new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_TOP_ID, MotorType.kBrushless);
    shooterBottomMotor =
        new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID, MotorType.kBrushless);

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

  // Spins up the shooter, and requests feeding it when the rollers are within parameters.
  // Receives distance-to-target from Limelight, or other sensor.
  // DOES NOT WORK CURRENTLY!
  public ParallelCommandGroup shooterFire() {
    addCommands(
      setShooterSpeeds(calculateShooterSpeeds(LimelightGetDistance())),
      isShooterReady(LimelightGetDistance())).withName("Shooter fire command");
  }

  // Idle shooter command; for default command purposes
  public Command shooterIdle() {
    return run(
        () -> {
          setShooterSpeeds(Constants.ShooterConstants.SHOOTER_IDLE_SPEEDS);
        }).withName("Idle shooter command");
  }

  // Checks if shooter is ready. 
  public void isShooterReady(double distance) {
    Pair<Double, Double> targetSpeeds = calculateShooterSpeeds(distance);
    double targetSpeedTop = targetSpeeds.getFirst();
    double targetSpeedBottom = targetSpeeds.getSecond();

    double minSpeedToleranceTop = targetSpeedTop * (1 - Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double maxSpeedToleranceTop = targetSpeedTop * (1 + Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double minSpeedToleranceBottom = targetSpeedBottom * (1 - Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double maxSpeedToleranceBottom = targetSpeedBottom * (1 + Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    double speedTop = shooterTopMotor.getEncoder().getVelocity();
    double speedBottom = shooterBottomMotor.getEncoder().getVelocity();

    if ((minSpeedToleranceTop < speedTop && speedTop < maxSpeedToleranceTop)
      && (minSpeedToleranceBottom < speedBottom && speedBottom < maxSpeedToleranceBottom) == true) {
        // Request note from feeder.
    }
  }

  // Speed in RPM. Top is index 0, bottom is index 1.
  public void setShooterSpeeds(Pair<Double, Double> speeds) {
    shooterTopMotor.getPIDController().setReference(speeds.getFirst(), CANSparkBase.ControlType.kVelocity);
    shooterBottomMotor
        .getPIDController()
        .setReference(speeds.getSecond(), CANSparkBase.ControlType.kVelocity);
  }

  // Top is index 0, bottom is index 1.
  public static Pair<Double, Double> calculateShooterSpeeds(double distance) {
    Pair<Double, Double> speeds;
    Double topSpeed = 0d;
    Double bottomSpeed = 0d;
    double distanceProportion;

    // If distance below minimum, set speed to minimum.
    if (distance < speakerConfigList[0].getDistance()) {
      topSpeed = speakerConfigList[0].getSpeedTop();
      bottomSpeed = speakerConfigList[0].getSpeedBottom();
    } else {
      // A linear search which determines which points the input distance falls between. May be
      // converted to a binary search if there are many points
      for (int i = 0; i < speakerConfigList.length; i++) {
        // If distance above maximum, set speed to maximum.
        if (i == speakerConfigList.length - 1) {
          topSpeed = speakerConfigList[i].getSpeedTop();
          bottomSpeed = speakerConfigList[i].getSpeedBottom();
          break;
        } else if (distance >= speakerConfigList[i].getDistance()
            && distance < speakerConfigList[i + 1].getDistance()) {
          // Math to linearly interpolate the speed.
          distanceProportion =
              (distance - speakerConfigList[i].getDistance())
                  / (speakerConfigList[i + 1].getDistance() - speakerConfigList[i].getDistance());
          topSpeed =
              (distanceProportion
                      * (speakerConfigList[i + 1].getSpeedTop()
                          - speakerConfigList[i].getSpeedTop()))
                  + speakerConfigList[i].getSpeedTop();
          bottomSpeed =
              (distanceProportion
                      * (speakerConfigList[i + 1].getSpeedBottom()
                          - speakerConfigList[i].getSpeedBottom()))
                  + speakerConfigList[i].getSpeedBottom();
          break;
        }
      }
    }

    speeds = new Pair(topSpeed, bottomSpeed);
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
