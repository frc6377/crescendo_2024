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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.utilities.DebugEntry;
import java.util.function.BooleanSupplier;

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

  private DebugEntry<Double> rightMotorOutputEntry;
  private DebugEntry<Double> rightMotorSpeedEntry;
  private DebugEntry<Double> rightMotorTargetSpeedEntry;
  private DebugEntry<Double> rightMotorTemperatureEntry;

  private DebugEntry<Double> leftFlywheelInputEntry;
  private DebugEntry<Double> leftFlywheelAngularVelocityEntry;

  private DebugEntry<Double> rightFlywheelInputEntry;
  private DebugEntry<Double> rightFlywheelAngularVelocityEntry;

  private DebugEntry<Boolean> shooterReadyEntry;

  private GenericEntry targetRPM = shooterTab.add("Target RPM", 0).getEntry();
  private GenericEntry rightTargetRPM = shooterTab.add("right RPM", 0).getEntry();
  private SpeakerConfig targetSpeeds;

  private FlywheelSim shooterLeftSim;
  private FlywheelSim shooterRightSim;

  public ShooterSubsystem() {
    shooterLeftMotor =
        new CANSparkMaxSim(Constants.ShooterConstants.SHOOTER_MOTOR_LEFT_ID, MotorType.kBrushless);
    shooterRightMotor =
        new CANSparkMaxSim(Constants.ShooterConstants.SHOOTER_MOTOR_RIGHT_ID, MotorType.kBrushless);

    shooterLeftMotor.restoreFactoryDefaults();
    shooterLeftMotor.setSmartCurrentLimit(40);
    shooterRightMotor.restoreFactoryDefaults();
    shooterRightMotor.setSmartCurrentLimit(40);

    shooterLeftMotor.setIdleMode(IdleMode.kCoast);
    shooterRightMotor.setIdleMode(IdleMode.kCoast);

    shooterLeftMotor.setInverted(true);

    shooterLeftMotor.getPIDController().setP(ShooterConstants.SHOOTER_LEFT_PID.getP());
    shooterLeftMotor.getPIDController().setI(ShooterConstants.SHOOTER_LEFT_PID.getI());
    shooterLeftMotor.getPIDController().setD(ShooterConstants.SHOOTER_LEFT_PID.getD());
    shooterLeftMotor.getPIDController().setIZone(ShooterConstants.SHOOTER_LEFT_PID.getIz());
    shooterLeftMotor.getPIDController().setFF(ShooterConstants.SHOOTER_LEFT_PID.getFF());
    shooterRightMotor.getPIDController().setP(ShooterConstants.SHOOTER_RIGHT_PID.getP());
    shooterRightMotor.getPIDController().setI(ShooterConstants.SHOOTER_RIGHT_PID.getI());
    shooterRightMotor.getPIDController().setD(ShooterConstants.SHOOTER_RIGHT_PID.getD());
    shooterRightMotor.getPIDController().setIZone(ShooterConstants.SHOOTER_RIGHT_PID.getIz());
    shooterRightMotor.getPIDController().setFF(ShooterConstants.SHOOTER_RIGHT_PID.getFF());

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

    rightMotorOutputEntry = new DebugEntry<Double>(0.0, "Right Motor Output", this);
    rightMotorSpeedEntry = new DebugEntry<Double>(0.0, "Right Motor Speed", this);
    rightMotorTargetSpeedEntry = new DebugEntry<Double>(0.0, "Right Motor Target Speed", this);
    rightMotorTemperatureEntry = new DebugEntry<Double>(0.0, "Right Motor Temperature", this);

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
  // Required to be called repeatedly; consider pub-sub for LimelightGetDistance() or equivalent
  // method to save a method call
  public Command revShooter() {
    return Commands.startEnd(
        () ->
            this.setShooterSpeeds(
                new SpeakerConfig(-1, targetRPM.getDouble(0), rightTargetRPM.getDouble(0))),
        () -> {},
        this);
  }

  public Command bumperShoot() {
    return run(() -> {
          setShooterSpeeds(speakerConfigList[0]);
        })
        .withName("Bumper shoot command");
  }

  // Idle shooter command; for default command purposes
  public Command shooterIdle() {
    return run(() -> {
          shooterLeftMotor.set(0);
          shooterRightMotor.set(0);
        })
        .withName("Idle Shooter command");
  }

  public Command setShooterIfReady(SpeakerConfig speeds, int exitCode) {
    return startEnd(
        () -> {
          if (exitCode == 0) {
            setShooterSpeeds(speeds);
          }
        },
        () -> {
          setShooterSpeeds(new SpeakerConfig(0, 0, 0));
          shooterLeftMotor.stopMotor();
          shooterRightMotor.stopMotor();
        });
  }

  public Trigger shooterReady() {
    return new Trigger(this::isShooterReady).debounce(0.05);
  }
  // Checks if shooter is ready.
  public boolean isShooterReady() {
    double targetSpeedLeft = targetSpeeds.getSpeedLeftInRPM();
    double targetSpeedRight = targetSpeeds.getSpeedRightInRPM();

    leftMotorTargetSpeedEntry.log(targetSpeedLeft);
    rightMotorTargetSpeedEntry.log(targetSpeedRight);

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
    new SpeakerConfig(0, 250, 250),
    new SpeakerConfig(40, 350, 350),
    new SpeakerConfig(195, 500, 500),
    new SpeakerConfig(290, 700, 700)
  };

  private static final SpeakerConfig speakerConfigIdle =
      new SpeakerConfig(
          -1,
          Constants.ShooterConstants.SHOOTER_IDLE_SPEED_LEFT,
          Constants.ShooterConstants.SHOOTER_IDLE_SPEED_RIGHT);

  public Command intakeSource() {
    return startEnd(
        () -> {
          setShooterSpeeds(ShooterConstants.SHOOTER_SOURCE_INTAKE);
        },
        () -> {});
  }

  public BooleanSupplier getBeamBreak() {
    return () -> false;
  }

  public Command intakeSourceForTime() {
    return Commands.deadline(new WaitCommand(ShooterConstants.INTAKE_DELAY_SEC), intakeSource());
  }
}
