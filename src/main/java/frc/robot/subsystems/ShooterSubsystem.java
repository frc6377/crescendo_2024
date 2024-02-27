package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.DebugEntry;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMaxSim shooterLeftMotor;
  private final CANSparkMaxSim shooterRightMotor;

  private final RelativeEncoder shooterLeftMotorEncoder;
  private final RelativeEncoder shooterRightMotorEncoder;

  private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterSubsystem");

  private DebugEntry<Double> leftMotorOutputEntry;
  private DebugEntry<Double> leftMotorSpeedEntry;
  private DebugEntry<Double> leftMotorTargetSpeedEntry;
  private DebugEntry<Double> leftMotorTemperatureEntry;
  private DebugEntry<Double> leftMotorCurrentEntry;

  private DebugEntry<Double> rightMotorOutputEntry;
  private DebugEntry<Double> rightMotorSpeedEntry;
  private DebugEntry<Double> rightMotorTargetSpeedEntry;
  private DebugEntry<Double> rightMotorTemperatureEntry;
  private DebugEntry<Double> rightMotorCurrentEntry;

  private DebugEntry<Double> leftFlywheelInputEntry;
  private DebugEntry<Double> leftFlywheelAngularVelocityEntry;

  private DebugEntry<Double> rightFlywheelInputEntry;
  private DebugEntry<Double> rightFlywheelAngularVelocityEntry;

  private DebugEntry<Boolean> shooterReadyEntry;

  private GenericEntry leftFeedForward =
      shooterTab
          .add("Left Motor Feed Forward", Constants.ShooterConstants.SHOOTER_LEFT_FF)
          .getEntry();
  private GenericEntry rightFeedForward =
      shooterTab
          .add("Right Motor Feed Forward", Constants.ShooterConstants.SHOOTER_RIGHT_FF)
          .getEntry();

  private GenericEntry leftShooterRPM = shooterTab.add("Shooter Left RPM", 4000).getEntry();
  private GenericEntry rightShooterRPM = shooterTab.add("Shooter Right RPM", 4000).getEntry();

  private SpeakerConfig targetSpeeds;

  private FlywheelSim shooterLeftSim;
  private FlywheelSim shooterRightSim;

  public ShooterSubsystem() {
    shooterLeftMotor =
        new CANSparkMaxSim(Constants.ShooterConstants.SHOOTER_MOTOR_LEFT_ID, MotorType.kBrushless);
    shooterRightMotor =
        new CANSparkMaxSim(Constants.ShooterConstants.SHOOTER_MOTOR_RIGHT_ID, MotorType.kBrushless);

    shooterLeftMotor.restoreFactoryDefaults();
    shooterLeftMotor.setSmartCurrentLimit(50);
    shooterRightMotor.restoreFactoryDefaults();
    shooterRightMotor.setSmartCurrentLimit(50);

    shooterLeftMotor.setIdleMode(IdleMode.kCoast);
    shooterRightMotor.setIdleMode(IdleMode.kCoast);

    shooterLeftMotor.setInverted(true);

    shooterLeftMotor.getPIDController().setP(Constants.ShooterConstants.SHOOTER_LEFT_P);
    shooterLeftMotor.getPIDController().setI(Constants.ShooterConstants.SHOOTER_LEFT_I);
    shooterLeftMotor.getPIDController().setD(Constants.ShooterConstants.SHOOTER_LEFT_D);
    shooterLeftMotor.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_LEFT_FF);
    shooterRightMotor.getPIDController().setP(Constants.ShooterConstants.SHOOTER_RIGHT_P);
    shooterRightMotor.getPIDController().setI(Constants.ShooterConstants.SHOOTER_RIGHT_I);
    shooterRightMotor.getPIDController().setD(Constants.ShooterConstants.SHOOTER_RIGHT_D);
    shooterRightMotor.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_RIGHT_FF);

    shooterTab.add("Shooter Left Motor PID", shooterLeftMotor.getPIDController());
    shooterTab.add("Shooter Right Motor PID", shooterRightMotor.getPIDController());

    if (Robot.isSimulation()) {
      shooterLeftSim =
          new FlywheelSim(
              DCMotor.getNEO(1),
              Constants.ShooterConstants.SHOOTER_LEFT_GEARING,
              Constants.ShooterConstants.SHOOTER_LEFT_MOMENT * 2); // 2 rollers
      shooterRightSim =
          new FlywheelSim(
              DCMotor.getNEO(1),
              Constants.ShooterConstants.SHOOTER_RIGHT_GEARING,
              Constants.ShooterConstants.SHOOTER_RIGHT_MOMENT * 2); // 2 rollers
    }

    shooterLeftMotorEncoder = shooterLeftMotor.getEncoder();
    shooterRightMotorEncoder = shooterRightMotor.getEncoder();

    targetSpeeds = new SpeakerConfig(0, 0, 0);

    leftMotorOutputEntry = new DebugEntry<Double>(0.0, "Left Motor Output", this);
    leftMotorSpeedEntry = new DebugEntry<Double>(0.0, "Left Motor Speed", this);
    leftMotorTargetSpeedEntry = new DebugEntry<Double>(0.0, "Left Motor Target Speed", this);
    leftMotorTemperatureEntry = new DebugEntry<Double>(0.0, "Left Motor Temperature", this);
    leftMotorCurrentEntry = new DebugEntry<Double>(0.0, "Left Motor Current", this);

    rightMotorOutputEntry = new DebugEntry<Double>(0.0, "Right Motor Output", this);
    rightMotorSpeedEntry = new DebugEntry<Double>(0.0, "Right Motor Speed", this);
    rightMotorTargetSpeedEntry = new DebugEntry<Double>(0.0, "Right Motor Target Speed", this);
    rightMotorTemperatureEntry = new DebugEntry<Double>(0.0, "Right Motor Temperature", this);
    rightMotorCurrentEntry = new DebugEntry<Double>(0.0, "Right Motor Current", this);

    leftFlywheelInputEntry = new DebugEntry<Double>(0.0, "Left Flywheel Input", this);
    leftFlywheelAngularVelocityEntry = new DebugEntry<Double>(0.0, "Left Flywheel RPM", this);

    rightFlywheelInputEntry = new DebugEntry<Double>(0.0, "Right Flywheel Input", this);
    rightFlywheelAngularVelocityEntry = new DebugEntry<Double>(0.0, "Right Flywheel RPM", this);

    shooterReadyEntry = new DebugEntry<Boolean>(false, "Shooter Ready?", this);
  }

  @Override
  public void periodic() {
    leftMotorSpeedEntry.log(shooterLeftMotorEncoder.getVelocity());
    rightMotorSpeedEntry.log(shooterRightMotorEncoder.getVelocity());

    leftMotorOutputEntry.log(shooterLeftMotor.getAppliedOutput());
    rightMotorOutputEntry.log(shooterRightMotor.getAppliedOutput());

    leftMotorTemperatureEntry.log(shooterLeftMotor.getMotorTemperature());
    rightMotorTemperatureEntry.log(shooterRightMotor.getMotorTemperature());

    leftMotorCurrentEntry.log(shooterLeftMotor.getOutputCurrent());
    rightMotorCurrentEntry.log(shooterRightMotor.getOutputCurrent());

    shooterLeftMotor
        .getPIDController()
        .setFF(leftFeedForward.getDouble(Constants.ShooterConstants.SHOOTER_LEFT_FF));
    shooterRightMotor
        .getPIDController()
        .setFF(rightFeedForward.getDouble(Constants.ShooterConstants.SHOOTER_RIGHT_FF));
  }

  @Override
  public void simulationPeriodic() {
    for (double i = 0; i < Robot.defaultPeriodSecs; i += CANSparkMaxSim.kPeriod) {
      shooterLeftSim.setInput(shooterLeftMotor.get() * RobotController.getBatteryVoltage());
      shooterLeftSim.update(CANSparkMaxSim.kPeriod);
      shooterLeftMotor.update(
          Units.rotationsPerMinuteToRadiansPerSecond(
              shooterLeftSim.getAngularVelocityRPM()
                  * Constants.ShooterConstants.SHOOTER_LEFT_GEARING));

      shooterRightSim.setInput(shooterRightMotor.get() * RobotController.getBatteryVoltage());
      shooterRightSim.update(CANSparkMaxSim.kPeriod);
      shooterRightMotor.update(
          Units.rotationsPerMinuteToRadiansPerSecond(
              shooterRightSim.getAngularVelocityRPM()
                  * Constants.ShooterConstants.SHOOTER_RIGHT_GEARING));

      leftFlywheelInputEntry.log(shooterLeftMotor.get() * RobotController.getBatteryVoltage());
      rightFlywheelInputEntry.log(shooterRightMotor.get() * RobotController.getBatteryVoltage());
    }
  }

  // Spins up the shooter, and requests feeding it when the rollers are within parameters.
  // Receives distance-to-target from Limelight, or other sensor.
  public Command revShooter() {
    return Commands.startEnd(
        () ->
            this.setShooterSpeeds( // Should receive distance-to-target from vision
                new SpeakerConfig(
                    -1, leftShooterRPM.getDouble(4000), rightShooterRPM.getDouble(4000))),
        () -> {
          shooterLeftMotor.stopMotor();
          shooterRightMotor.stopMotor();
        },
        this);
  }

  public Command bumperShoot() {
    return startEnd(
            () -> {
              setShooterSpeeds(speakerConfigList[0]);
            },
            () -> {
              shooterLeftMotor.stopMotor();
              shooterRightMotor.stopMotor();
              leftMotorTargetSpeedEntry.log(0d);
              rightMotorTargetSpeedEntry.log(0d);
            })
        .withName("Bumper shoot command");
  }

  // Idle shooter command; for default command purposes
  public Command shooterIdle() {
    // TODO: Determine appropriate open loop rate for shooter idle
    return run(() -> {
          setShooterSpeeds(speakerConfigIdle);
        })
        .withName("Idle Shooter command");
  }

  public Trigger shooterReady() {
    return new Trigger(this::isShooterReady).debounce(0.05);
  }

  // Checks if shooter is ready.
  public boolean isShooterReady() {
    double targetSpeedLeft = targetSpeeds.getSpeedLeftInRPM();
    double targetSpeedRight = targetSpeeds.getSpeedRightInRPM();

    double minSpeedToleranceLeft =
        targetSpeedLeft * (1 - Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double minSpeedToleranceRight =
        targetSpeedRight * (1 - Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    double maxSpeedToleranceLeft =
        targetSpeedLeft * (1 + Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    double maxSpeedToleranceRight =
        targetSpeedRight * (1 + Constants.ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    double speedLeft = shooterLeftMotorEncoder.getVelocity();
    double speedRight = shooterRightMotorEncoder.getVelocity();

    boolean ready =
        (minSpeedToleranceLeft < speedLeft && speedLeft < maxSpeedToleranceLeft)
            && (minSpeedToleranceRight < speedRight && speedRight < maxSpeedToleranceRight);
    shooterReadyEntry.log(ready);
    return ready;
  }

  // Speed in RPM. Left is index 0, right is index 1.
  public SpeakerConfig setShooterSpeeds(SpeakerConfig speeds) {
    targetSpeeds = speeds;

    leftMotorTargetSpeedEntry.log(targetSpeeds.getSpeedLeftInRPM());
    rightMotorTargetSpeedEntry.log(targetSpeeds.getSpeedRightInRPM());

    shooterLeftMotor
        .getPIDController()
        .setReference(speeds.getSpeedLeftInRPM(), CANSparkBase.ControlType.kVelocity);
    shooterRightMotor
        .getPIDController()
        .setReference(speeds.getSpeedRightInRPM(), CANSparkBase.ControlType.kVelocity);

    return targetSpeeds;
  }

  // Left is index 0, right is index 1.
  public static SpeakerConfig calculateShooterSpeeds(double distance) {
    SpeakerConfig speeds;
    Double leftSpeed = 0d;
    Double rightSpeed = 0d;
    double distanceProportion;

    // If distance below minimum, set speed to minimum.
    if (distance < speakerConfigList[0].getDistanceInInches()) {
      leftSpeed = speakerConfigList[0].getSpeedLeftInRPM();
      rightSpeed = speakerConfigList[0].getSpeedRightInRPM();
      speeds = new SpeakerConfig(distance, leftSpeed, rightSpeed);
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
        leftSpeed =
            (distanceProportion
                    * (speakerConfigList[i + 1].getSpeedLeftInRPM()
                        - speakerConfigList[i].getSpeedLeftInRPM()))
                + speakerConfigList[i].getSpeedLeftInRPM();
        rightSpeed =
            (distanceProportion
                    * (speakerConfigList[i + 1].getSpeedRightInRPM()
                        - speakerConfigList[i].getSpeedRightInRPM()))
                + speakerConfigList[i].getSpeedRightInRPM();
        speeds = new SpeakerConfig(distance, leftSpeed, rightSpeed);

        return speeds;
      }
    }

    // If distance above maximum, set speed to maximum.
    leftSpeed = speakerConfigList[speakerConfigList.length - 1].getSpeedLeftInRPM();
    rightSpeed = speakerConfigList[speakerConfigList.length - 1].getSpeedRightInRPM();
    speeds = new SpeakerConfig(distance, leftSpeed, rightSpeed);
    return speeds;
  }

  // Distance and speed in inches and RPM respectively.
  public static class SpeakerConfig {
    private double distanceInInches;
    private double speedLeftInRPM;
    private double speedRightInRPM;

    public SpeakerConfig(double distance, double speedLeft, double speedRight) {
      this.distanceInInches = distance;
      this.speedLeftInRPM = speedLeft;
      this.speedRightInRPM = speedRight;
    }

    public double getDistanceInInches() {
      return distanceInInches;
    }

    // Motor RPM, NOT roller RPM
    public double getSpeedLeftInRPM() {
      return speedLeftInRPM;
    }

    // Motor RPM, NOT roller RPM
    public double getSpeedRightInRPM() {
      return speedRightInRPM;
    }
  }

  // Motor RPM, NOT roller RPM
  private static SpeakerConfig[] speakerConfigList = {
    new SpeakerConfig(0, 2350, 1950),
    new SpeakerConfig(40, 2550, 2150),
    new SpeakerConfig(195, 2750, 2350),
    new SpeakerConfig(290, 2750, 2350)
  };

  private static final SpeakerConfig speakerConfigIdle =
      new SpeakerConfig(
          -1,
          Constants.ShooterConstants.SHOOTER_IDLE_SPEED_LEFT,
          Constants.ShooterConstants.SHOOTER_IDLE_SPEED_RIGHT);
}
