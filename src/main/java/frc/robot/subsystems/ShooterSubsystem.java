package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.DebugEntry;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMaxSim shooterTopMotor;
  private final CANSparkMaxSim shooterBottomMotor;

  private final RelativeEncoder shooterTopMotorEncoder;
  private final RelativeEncoder shooterBottomMotorEncoder;

  private DebugEntry<Double> topMotorSpeedEntry;
  private DebugEntry<Double> bottomMotorSpeedEntry;
  private DebugEntry<Double> topMotorTargetSpeedEntry;
  private DebugEntry<Double> bottomMotorTargetSpeedEntry;

  public ShooterSubsystem() {
    shooterTopMotor =
        new CANSparkMaxSim(Constants.ShooterConstants.SHOOTER_MOTOR_TOP_ID, MotorType.kBrushless);
    shooterBottomMotor =
        new CANSparkMaxSim(
            Constants.ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID, MotorType.kBrushless);

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

    shooterTopMotorEncoder = shooterTopMotor.getEncoder();
    shooterBottomMotorEncoder = shooterBottomMotor.getEncoder();
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
          setShooterSpeeds(speakerConfigIdle);
        })
        .withName("Idle shooter command");
  }

  // Checks if shooter is ready.
  public void isShooterReady(SpeakerConfig targetSpeeds) {
    double targetSpeedTop = targetSpeeds.getSpeedTopInRPM();
    double targetSpeedBottom = targetSpeeds.getSpeedBottomInRPM();

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

    double speedTop = shooterTopMotorEncoder.getVelocity();
    double speedBottom = shooterBottomMotorEncoder.getVelocity();

    topMotorSpeedEntry.log(speedTop);
    bottomMotorSpeedEntry.log(speedBottom);

    if ((minSpeedToleranceTop < speedTop && speedTop < maxSpeedToleranceTop)
        && (minSpeedToleranceBottom < speedBottom && speedBottom < maxSpeedToleranceBottom)
            == true) {
      // Request note from feeder.
    }
  }

  // Speed in RPM. Top is index 0, bottom is index 1.
  public SpeakerConfig setShooterSpeeds(SpeakerConfig speeds) {
    SpeakerConfig targetSpeeds = speeds;

    shooterTopMotor
        .getPIDController()
        .setReference(speeds.getSpeedTopInRPM(), CANSparkBase.ControlType.kVelocity);
    shooterBottomMotor
        .getPIDController()
        .setReference(speeds.getSpeedBottomInRPM(), CANSparkBase.ControlType.kVelocity);

    return targetSpeeds;
  }

  // Top is index 0, bottom is index 1.
  public static SpeakerConfig calculateShooterSpeeds(double distance) {
    SpeakerConfig speeds;
    Double topSpeed = 0d;
    Double bottomSpeed = 0d;
    double distanceProportion;

    // If distance below minimum, set speed to minimum.
    if (distance < speakerConfigList[0].getDistanceInInches()) {
      topSpeed = speakerConfigList[0].getSpeedTopInRPM();
      bottomSpeed = speakerConfigList[0].getSpeedBottomInRPM();
      speeds = new SpeakerConfig(distance, topSpeed, bottomSpeed);
      return speeds;
    }
    // A linear search which determines which points the input distance falls between. May be
    // converted to a binary search if there are many points.
    // Loop exits before the last element because interpolation cannot be done on the last element.
    for (int i = 0; i <= speakerConfigList.length - 2; i++) {
      if (distance >= speakerConfigList[i].getDistanceInInches()
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
        speeds = new SpeakerConfig(distance, topSpeed, bottomSpeed);
        System.out.println(i + " " + distance);
        return speeds;
      }
    }

    // If distance above maximum, set speed to maximum.
    topSpeed = speakerConfigList[speakerConfigList.length - 1].getSpeedTopInRPM();
    bottomSpeed = speakerConfigList[speakerConfigList.length - 1].getSpeedBottomInRPM();
    speeds = new SpeakerConfig(distance, topSpeed, bottomSpeed);
    return speeds;
  }

  // Distance and speed in inches and RPM respectively.
  public static class SpeakerConfig {
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

  private static SpeakerConfig speakerConfigIdle =
      new SpeakerConfig(
          -1,
          Constants.ShooterConstants.SHOOTER_IDLE_SPEED_TOP,
          Constants.ShooterConstants.SHOOTER_IDLE_SPEED_BOTTOM);

  public class SetShooter extends Command {
    SpeakerConfig shooterSpeeds;
    int exitCode;

    public SetShooter(SpeakerConfig speeds, int exitCode) {
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
