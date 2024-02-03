package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.DebugEntry;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax shooterTopMotor;
  private final CANSparkMax shooterBottomMotor;

  private DebugEntry<Double> topMotorSpeedEntry;
  private DebugEntry<Double> bottomMotorSpeedEntry;
  private DebugEntry<Double> topMotorTargetSpeedEntry;
  private DebugEntry<Double> bottomMotorTargetSpeedEntry;

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
  // Required to be called repeatedly; consider pub-sub for LimelightGetDistance() or equivalent
  // method to save a method call
  public Command shooterFire() {
    return Commands.sequence(
        // Only runs if the exit code from the limelight status function returns 0!
        new SetShooter(
            calculateShooterSpeeds(0),
            0)); // Replace distance and exit code with LimelightGetDistance() and
    // CheckLimelightStatus() respectively
  }

  // Idle shooter command; for default command purposes
  public Command shooterIdle() {
    return run(() -> {
          setShooterSpeeds(Constants.ShooterConstants.SHOOTER_IDLE_SPEEDS);
        })
        .withName("Idle shooter command");
  }

  // Checks if shooter is ready.
  public void isShooterReady(Pair<Double, Double> targetSpeeds) {
    double targetSpeedTop = targetSpeeds.getFirst();
    double targetSpeedBottom = targetSpeeds.getSecond();

    topMotorTargetSpeedEntry.log(targetSpeedTop);
    bottomMotorTargetSpeedEntry.log(targetSpeedBottom);

    double minSpeedToleranceTop =
        targetSpeedTop * (1 - Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double maxSpeedToleranceTop =
        targetSpeedTop * (1 + Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double minSpeedToleranceBottom =
        targetSpeedBottom * (1 - Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double maxSpeedToleranceBottom =
        targetSpeedBottom * (1 + Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    double speedTop = shooterTopMotor.getEncoder().getVelocity();
    double speedBottom = shooterBottomMotor.getEncoder().getVelocity();

    topMotorSpeedEntry.log(speedTop);
    bottomMotorSpeedEntry.log(speedBottom);

    if ((minSpeedToleranceTop < speedTop && speedTop < maxSpeedToleranceTop)
        && (minSpeedToleranceBottom < speedBottom && speedBottom < maxSpeedToleranceBottom)
            == true) {
      // Request note from feeder.
    }
  }

  // Speed in RPM. Top is index 0, bottom is index 1.
  public Pair<Double, Double> setShooterSpeeds(Pair<Double, Double> speeds) {
    Pair<Double, Double> targetSpeeds = speeds;

    shooterTopMotor
        .getPIDController()
        .setReference(speeds.getFirst(), CANSparkBase.ControlType.kVelocity);
    shooterBottomMotor
        .getPIDController()
        .setReference(speeds.getSecond(), CANSparkBase.ControlType.kVelocity);

    return targetSpeeds;
  }

  // Top is index 0, bottom is index 1.
  public static Pair<Double, Double> calculateShooterSpeeds(double distance) {
    Pair<Double, Double> speeds;
    Double topSpeed = 0d;
    Double bottomSpeed = 0d;
    double distanceProportion;

    // If distance below minimum, set speed to minimum.
    if (distance < speakerConfigList[0].getDistanceInInches()) {
      topSpeed = speakerConfigList[0].getSpeedTopInRPM();
      bottomSpeed = speakerConfigList[0].getSpeedBottomInRPM();
    } else {
      // A linear search which determines which points the input distance falls between. May be
      // converted to a binary search if there are many points
      for (int i = 0; i < speakerConfigList.length; i++) {
        // If distance above maximum, set speed to maximum.
        if (i == speakerConfigList.length - 1) {
          topSpeed = speakerConfigList[i].getSpeedTopInRPM();
          bottomSpeed = speakerConfigList[i].getSpeedBottomInRPM();
          break;
        } else if (distance >= speakerConfigList[i].getDistanceInInches()
            && distance < speakerConfigList[i + 1].getDistanceInInches()) {
          // Math to linearly interpolate the speed.
          distanceProportion =
              (distance - speakerConfigList[i].getDistanceInInches())
                  / (speakerConfigList[i + 1].getDistanceInInches()
                      - speakerConfigList[i].getDistanceInInches());
          topSpeed =
              (distanceProportion
                      * (speakerConfigList[i + 1].getSpeedTopInRPM()
                          - speakerConfigList[i].getSpeedTopInRPM()))
                  + speakerConfigList[i].getSpeedTopInRPM();
          bottomSpeed =
              (distanceProportion
                      * (speakerConfigList[i + 1].getSpeedBottomInRPM()
                          - speakerConfigList[i].getSpeedBottomInRPM()))
                  + speakerConfigList[i].getSpeedBottomInRPM();
          break;
        }
      }
    }

    speeds = new Pair(topSpeed, bottomSpeed);
    return speeds;
  }

  // Distance and speed in inches and RPM respectively.
  private static class SpeakerConfig {
    private double distanceInInches;
    private double speedTopInRPM;
    private double speedBottomInRPM;

    public SpeakerConfig(double distance, double speedTop, double speedBottom) {
      this.distanceInInches = distance;
      this.speedTopInRPM = speedTop;
      this.speedBottomInRPM = speedBottom;
    }

    public double getDistanceInInches() {
      return distanceInInches;
    }

    public double getSpeedTopInRPM() {
      return speedTopInRPM;
    }

    public double getSpeedBottomInRPM() {
      return speedBottomInRPM;
    }
  }

  private static SpeakerConfig[] speakerConfigList = {
    new SpeakerConfig(0, 450, 250),
    new SpeakerConfig(40, 550, 350),
    new SpeakerConfig(195, 750, 500),
    new SpeakerConfig(290, 1000, 700)
  };

  public class SetShooter extends Command {
    Pair<Double, Double> shooterSpeeds;
    int exitCode;

    public SetShooter(Pair<Double, Double> speeds, int exitCode) {
      this.shooterSpeeds = speeds;
      this.exitCode = exitCode;
    }

    public void execute() {
      // Only runs if the exit code from the limelight distance function returns 0!
      if (exitCode == 0) {
        setShooterSpeeds(shooterSpeeds);
        isShooterReady(shooterSpeeds);
      }
    }
  }
}
