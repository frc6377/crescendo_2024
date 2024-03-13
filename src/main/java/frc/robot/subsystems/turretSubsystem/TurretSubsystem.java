// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turretSubsystem;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.config.TurretZeroConfig;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.HowdyMath;
import frc.robot.utilities.TunableNumber;

public class TurretSubsystem extends SubsystemBase {
  private final CANSparkMax turretMotor;
  private final PIDController turretPIDController;

  private final CANSparkMaxSim pitchMotor;
  private final PIDController pitchPIDController;
  private final ArmFeedforward pitchFeedForward;

  private final CANcoder highGearCANcoder;
  private final CANcoder lowGearCANcoder;
  private final SparkAbsoluteEncoder pitchEncoder;

  private double turretPosition;
  private double turretVelocity;
  private double pitchPosition = 10;
  private double pitchVelocity;

  private SingleJointedArmSim turretSim;
  private Mechanism2d turretMech;
  private MechanismRoot2d turretRoot;
  private MechanismLigament2d turretAngleSim;

  private SingleJointedArmSim pitchSim;
  private Mechanism2d pitchMech;
  private MechanismRoot2d pitchRoot;
  private MechanismLigament2d pitchAngleSim;

  private SimDeviceSim simTurretEncoder;
  private SimDouble simTurretPos;

  private InterpolatingDoubleTreeMap pitchMap;

  private TunableNumber pitchTune;
  private double pitchValue = 0;

  private final ShuffleboardTab turretTab = Shuffleboard.getTab(this.getName());
  private final DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Turret Position (Rotations)", this);
  private final DebugEntry<Double> turretGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Turret Goal Position (Radians)", this);
  private final DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Turret Velocity (RPM)", this);
  private final DebugEntry<Double> pitchPositionEntry =
      new DebugEntry<Double>(pitchPosition, "Pitch Position (Rotations)", this);
  private final DebugEntry<Double> pitchGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Pitch Goal Position (Radians)", this);
  private final DebugEntry<Double> pitchVelocityEntry =
      new DebugEntry<Double>(pitchVelocity, "Pitch Velocity (RPM)", this);
  private final DebugEntry<Double> motorOutputEntry =
      new DebugEntry<Double>(0.0, "Turret Motor Output", this);
  private final DebugEntry<Double> tagDistanceEntry =
      new DebugEntry<Double>(0.0, "LastMeasuredTagDistance", this);
  private DebugEntry<String> currentCommand =
      new DebugEntry<String>("none", "Turret Command", this);
  private DebugEntry<Double> pitchMotorOutput =
      new DebugEntry<Double>(0.0, "Pitch Motor Output", this);

  private final RobotStateManager robotStateManager;
  private final VisionSubsystem visionSubsystem;

  public TurretSubsystem(RobotStateManager robotStateManager, VisionSubsystem visionSubsystem) {
    // Initialize Motors
    turretMotor = new CANSparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setInverted(Constants.TurretConstants.IS_MOTOR_INVERTED);
    turretMotor.setSmartCurrentLimit(Constants.TurretConstants.TURRET_SMART_CURRENT_LIMIT);
    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse,
        (float) TurretConstants.TURRET_MIN_ANGLE_ROTATIONS);
    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward,
        (float) TurretConstants.TURRET_MAX_ANGLE_ROTATIONS);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    turretPIDController = TurretConstants.TURRET_PID.getPIDController();
    TurretConstants.TURRET_PID.createTunableNumbers("Turret Motor", turretPIDController, this);

    pitchMotor = new CANSparkMaxSim(Constants.TurretConstants.PITCH_MOTOR_ID, MotorType.kBrushless);
    pitchMotor.restoreFactoryDefaults();
    pitchMotor.setSmartCurrentLimit(Constants.TurretConstants.PITCH_SMART_CURRENT_LIMIT);
    pitchMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, (float) TurretConstants.PITCH_MIN_ANGLE_ROTATIONS);
    pitchMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward, (float) TurretConstants.PITCH_MAX_ANGLE_ROTATIONS);
    pitchMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    pitchMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    pitchPIDController = TurretConstants.PITCH_PID.getPIDController();
    TurretConstants.PITCH_PID.createTunableNumbers("Pitch Motor", pitchPIDController, this);
    pitchFeedForward = TurretConstants.PITCH_FF.getArmFeedforward();

    // Encoders
    highGearCANcoder = new CANcoder(Constants.TurretConstants.highGearCAN_CODER_ID);
    lowGearCANcoder = new CANcoder(Constants.TurretConstants.lowGearCAN_CODER_ID);
    pitchEncoder = pitchMotor.getAbsoluteEncoder();

    TurretZeroConfig zeroConfig = new DynamicRobotConfig().getTurretZeroConfig();
    MagnetSensorConfigs highGearSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(zeroConfig.highGearTurretZero);
    MagnetSensorConfigs lowGearSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(zeroConfig.lowGearTurretZero);

    highGearCANcoder.getConfigurator().apply(highGearSensorConfigs);
    lowGearCANcoder.getConfigurator().apply(lowGearSensorConfigs);

    if (Robot.isReal()) {
      zeroTurret();
    }

    this.robotStateManager = robotStateManager;
    this.visionSubsystem = visionSubsystem;

    pitchTune = new TunableNumber("pitch GoTo Value", 0.0, (a) -> pitchValue = a, this);

    pitchMap = new InterpolatingDoubleTreeMap();
    pitchMap.put(0.0, 0.0);
    pitchMap.put(1.0, 1.0);

    // Simulation
    if (Robot.isSimulation()) {
      // Turret
      simTurretEncoder =
          new SimDeviceSim(
              "CANEncoder:CANCoder (v6)", Constants.TurretConstants.highGearCAN_CODER_ID);
      simTurretPos = simTurretEncoder.getDouble("rawPositionInput");
      turretSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              1 / Constants.TurretConstants.TURRET_MOTOR_TURRET_RATIO,
              3.5 * 0.1016 * 0.1016 / 3,
              0.1016,
              -Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES),
              Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES),
              false,
              0);
      turretMech = new Mechanism2d(4, 4);
      turretRoot = turretMech.getRoot("Root", 2, 2);
      turretAngleSim =
          turretRoot.append(new MechanismLigament2d("Turret", 2, 0, 5, new Color8Bit(Color.kRed)));
      turretTab.add("Turret", turretMech);

      // pitch
      pitchSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              Constants.TurretConstants.PITCH_CONVERSION_FACTOR,
              Constants.TurretConstants.SHOOTER_MASS
                  * Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY
                  * Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY
                  / 3,
              Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY / 2.0,
              Math.toRadians(Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES),
              Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES),
              true,
              0);
      pitchMech = new Mechanism2d(4, 4);
      pitchRoot = pitchMech.getRoot("Root", 2, 2);
      pitchAngleSim =
          pitchRoot.append(new MechanismLigament2d("Pitch", 2, 0, 5, new Color8Bit(Color.kBlue)));
      turretTab.add("Pitch", pitchMech);
    }
  }

  public void stopTurret() {

    turretMotor.stopMotor();
  }

  public double calculateTurretPosition() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    double encoderPosition =
        lowGearCANcoder.getPosition().getValueAsDouble()
            / Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO;
    return (encoderPosition + Constants.TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV)
        * Math.PI
        * 2;
  }

  private double calculatePitchPosition() {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return 0;
    return Math.toRadians(
        (pitchEncoder.getPosition() * 360) * Constants.TurretConstants.PITCH_CONVERSION_FACTOR);
  }

  /** Will calculate the current turret position and update encoders and motors off of it. */
  public void zeroTurret() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return;
    double lowGearPosition = lowGearCANcoder.getAbsolutePosition().getValue().doubleValue();
    double highGearPosition = highGearCANcoder.getAbsolutePosition().getValue().doubleValue();
    Rotation2d turretRotation = encoderPositionsToTurretRotation(lowGearPosition, highGearPosition);

    lowGearCANcoder.setPosition(
        turretRotation.getRotations() * Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO);
    highGearCANcoder.setPosition(
        turretRotation.getRotations() * Constants.TurretConstants.HIGH_GEAR_CAN_CODER_RATIO);
    turretMotor
        .getEncoder()
        .setPosition(
            turretRotation.getRotations() * Constants.TurretConstants.TURRET_MOTOR_TURRET_RATIO);
  }

  /**
   * Calculates the turret zero off of given encoder rotations.
   *
   * @param lowGearCANcoderPosition The low gear encoder position, as in the gear with a lower gear
   *     ratio
   * @param highGearCANcoderPosition The high gear encoder position, as in the gear with a higher
   *     gear ratio
   * @return the calculated turret rotation
   */
  public static Rotation2d encoderPositionsToTurretRotation(
      double lowGearCANcoderPosition, double highGearCANcoderPosition) {
    // This equation is based off of
    // https://www.geeksforgeeks.org/implementation-of-chinese-remainder-theorem-inverse-modulo-based-implementation/
    // It is accurate to with in 3.6 deg
    int gearToothPosition =
        ((int) (lowGearCANcoderPosition * Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH)
                    * HowdyMath.inverse_modulus(
                        Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH,
                        Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH)
                    * Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH
                + (int)
                        (highGearCANcoderPosition
                            * Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH)
                    * HowdyMath.inverse_modulus(
                        Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH,
                        Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH)
                    * Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH)
            % (Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH
                * Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH);

    double roughRotation = gearToothPosition / (Constants.TurretConstants.TURRET_GEAR_TEETH + 0.0);
    double lowGearCANCoderDivsionSize =
        (Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH + 0.0)
            / Constants.TurretConstants.TURRET_GEAR_TEETH;
    double highGearCANCoderDivsionSize =
        (Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH + 0.0)
            / Constants.TurretConstants.TURRET_GEAR_TEETH;

    double distToLowGearCanCoderDivide =
        Math.abs(0.5 - (roughRotation / lowGearCANCoderDivsionSize) % 1);
    double distToHighGearCanCoderDivide =
        Math.abs(0.5 - (roughRotation / highGearCANCoderDivsionSize) % 1);

    double position;
    if (distToLowGearCanCoderDivide < distToHighGearCanCoderDivide) {
      // use low gear CanCoder for fine zeroing
      position =
          fineTuneTurretRotation(
              roughRotation, lowGearCANCoderDivsionSize, lowGearCANcoderPosition);
    } else {
      // use high gear CanCoder for fine zeroing
      position =
          fineTuneTurretRotation(
              roughRotation, highGearCANCoderDivsionSize, highGearCANcoderPosition);
    }

    return Rotation2d.fromRotations(position);
  }

  private static double fineTuneTurretRotation(
      double roughPosition, double divisionSize, double CANCoderAngle) {

    int division = (int) ((roughPosition / divisionSize));
    return divisionSize * division + CANCoderAngle * divisionSize;
  }

  public void setTurretPos(double setpoint) {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return;

    turretGoalPositionEntry.log(setpoint);
    turretMotor.set(
        turretPIDController.calculate(
            turretPosition,
            MathUtil.clamp(
                setpoint,
                Math.toRadians(Constants.TurretConstants.TURRET_MIN_ANGLE_DEGREES),
                Math.toRadians(Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES))));
  }

  public void setPitchPos(double setpoint) {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return;

    pitchGoalPositionEntry.log(setpoint);
    pitchMotor.setVoltage(
        pitchPIDController.calculate(
                pitchPosition,
                MathUtil.clamp(
                    setpoint,
                    Math.toRadians(Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES),
                    Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES)))
            + pitchFeedForward.calculate(pitchPosition, 0));
  }

  public void holdPosition() {
    if (Constants.enabledSubsystems.turretRotationEnabled) setTurretPos(0);
    if (Constants.enabledSubsystems.turretPitchEnabled) setPitchPos(0);
  }

  public void moveUp() {
    if (Constants.enabledSubsystems.turretRotationEnabled) setTurretPos(turretPosition);
    if (Constants.enabledSubsystems.turretPitchEnabled) setPitchPos(pitchValue);
  }

  public void updateTurretPosition() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return;
    turretPosition = calculateTurretPosition();
  }

  private void updatePitchPosition() {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return;
    pitchPosition = calculatePitchPosition();
  }

  public double getTurretPos() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    return turretPosition; // returns the absolute encoder position in radians
  }

  public double getTurretVel() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    return turretVelocity;
  }

  public void aimTurret() {
    if (visionSubsystem != null) {
      int tagID =
          ((robotStateManager.getAllianceColor()
                  == AllianceColor
                      .BLUE) // Default to red because that's the color on our test field
              ? Constants.TurretConstants.SPEAKER_TAG_ID_BLUE
              : Constants.TurretConstants.SPEAKER_TAG_ID_RED);
      double visionTX = visionSubsystem.getTurretYaw(tagID);
      if (visionTX != 0) {
        // X & Rotation
        if (Constants.enabledSubsystems.turretRotationEnabled) {
          setTurretPos(Math.toRadians(visionTX) + turretPosition);
        }

        // Y & Tilting
        if (Constants.enabledSubsystems.turretPitchEnabled) {
          double visionTY = visionSubsystem.getTurretPitch(tagID);
          double distanceToTag = tyToDistanceFromTag(visionTY);
          tagDistanceEntry.log(distanceToTag);
          setPitchPos(distanceToShootingPitch(tyToDistanceFromTag(visionTY)));
        }

        if (Math.abs(Math.toRadians(visionTX) + turretPosition)
            > Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES)) {
          // TODO: Make turret rotate the drivebase if necessary and driver thinks it's a good idea
        }
      }
    } else {
      // TODO: Make turret default to using odometry
      // If we don't see any tags it might mean we're right next to the speaker so it'll go to
      // default shooting position
      if (Constants.enabledSubsystems.turretRotationEnabled) {
        setTurretPos(Constants.TurretConstants.DEFAULT_SHOT_ROTATION);
      }
      if (Constants.enabledSubsystems.turretPitchEnabled) {
        setPitchPos(Constants.TurretConstants.DEFAULT_SHOT_PITCH);
      }
    }
  }

  private void aimTurretOdometry(Pose2d robotPos, Pose2d targetPos) {
    setTurretPos(getTurretRotationFromOdometry(robotPos, targetPos));
  }

  /**
   * Calculates the distance to the tag centered on the speaker from the angle that the limelight
   * sees it
   *
   * @param ty The ty output by the limelight (degrees)
   * @return The distance from the tag (meters)
   */
  private double tyToDistanceFromTag(double ty) {
    DynamicRobotConfig dynamicRobotConfig = new DynamicRobotConfig();
    double tagTheta =
        Math.toRadians(ty) + dynamicRobotConfig.getLimelightConfig().limelightPitchRadians;
    double height =
        Constants.TurretConstants.SPEAKER_TAG_CENTER_HEIGHT_METERS
            - dynamicRobotConfig.getLimelightConfig()
                .limelightYMeters; // TODO: Change these alphabot constants to be turret
    // constants whenever the robot is built
    double distance = height / Math.tan(tagTheta);
    return distance;
  }

  /**
   * Calculates the pitch the shooter should be angled at to fire a given distance.
   *
   * @param distance The distance from the speaker in meters
   * @return The shooter pitch in degrees
   */
  private double distanceToShootingPitch(double distance) {
    return 4; // TODO: Make and use a real formula(use testing, not physics)
  }

  @Override
  public void periodic() {
    updateTurretPosition();
    turretVelocity =
        (lowGearCANcoder.getVelocity().getValueAsDouble()
                * Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO)
            * 60; // changing from rotations per second to rotations per minute or rpm
    turretPositionEntry.log(turretPosition);
    turretVelocityEntry.log(turretVelocity);

    updatePitchPosition();
    pitchVelocity =
        pitchEncoder.getVelocity()
            * 60; // changing from rotations per second to rotations per minute or rpm
    pitchPositionEntry.log(pitchPosition);
    pitchVelocityEntry.log(pitchVelocity);

    motorOutputEntry.log(turretMotor.get());
    pitchMotorOutput.log(pitchMotor.get());

    if (this.getCurrentCommand() != null) currentCommand.log(this.getCurrentCommand().getName());
  }

  @Override
  public void simulationPeriodic() {
    turretSim.setInput(turretMotor.get() * RobotController.getBatteryVoltage());
    turretSim.update(Robot.defaultPeriodSecs);
    turretAngleSim.setAngle(Math.toDegrees(turretSim.getAngleRads()));
    simTurretPos.set(
        Units.radiansToRotations(
            turretSim.getAngleRads() * Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO));

    pitchSim.setInput(pitchMotor.getAppliedOutput());
    pitchSim.update(Robot.defaultPeriodSecs);
    pitchAngleSim.setAngle(Math.toDegrees(pitchSim.getAngleRads()));
    pitchMotor.setAbsolutePosition(Units.radiansToRotations(pitchSim.getAngleRads()));
  }

  private double getTurretRotationFromOdometry(Pose2d robotPos, Pose2d targetPos) {
    return Math.atan2(robotPos.getY() - targetPos.getY(), robotPos.getX() - targetPos.getX())
        + robotPos.getRotation().getRadians();
  }

  protected CANcoder getLowGearCANCoder() {
    return lowGearCANcoder;
  }

  protected CANcoder getHighGearCANCoder() {
    return highGearCANcoder;
  }
}
