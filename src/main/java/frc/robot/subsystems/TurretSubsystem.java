// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.config.TurretZeroConfig;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.HowdyMath;
import frc.robot.utilities.LimelightHelpers;
import java.util.function.Consumer;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax turretMotor;
  private SingleJointedArmSim turretSim;
  private Mechanism2d turretMech;
  private MechanismRoot2d turretRoot;
  private MechanismLigament2d turretAngleSim;
  private final ShuffleboardTab turretTab = Shuffleboard.getTab("Turret Tab");
  private final SimDeviceSim simTurretEncoder;
  private final SimDouble simTurretPos;

  private final PIDController turretPIDController;
  private final CANcoder highGearCANcoder;
  private final CANcoder lowGearCANcoder;
  private double turretPosition;
  private double turretVelocity;
  private Consumer<Double> turretTestPosition;
  private final ShuffleboardTab configTab = Shuffleboard.getTab("Config");
  private final GenericEntry turretKP =
      configTab.add("Turret KP", Constants.TurretConstants.TURRET_KP).getEntry();
  private final GenericEntry turretKI =
      configTab.add("Turret KI", Constants.TurretConstants.TURRET_KI).getEntry();
  private final GenericEntry turretKD =
      configTab.add("Turret KD", Constants.TurretConstants.TURRET_KD).getEntry();
  private final DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Turret Position", this);
  private final DebugEntry<Double> turretGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Turret Goal Position", this);
  private final DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Turret Velocity", this);

  private final CANSparkMax pitchMotor;
  private SingleJointedArmSim pitchSim;
  private Mechanism2d pitchMech;
  private MechanismRoot2d pitchRoot;
  private MechanismLigament2d pitchAngleSim;
  private final SimDeviceSim simPitchEncoder;
  private final SimDouble simPitchPos;
  private final ArmFeedforward pitchFeedForward;

  private final PIDController pitchPIDController;
  private final SparkAbsoluteEncoder pitchEncoder;
  private double pitchPosition;
  private double pitchVelocity;
  private Consumer<Double> pitchTestPosition;
  private final GenericEntry pitchKP =
      configTab.add("Pitch KP", Constants.TurretConstants.PITCH_KP).getEntry();
  private final GenericEntry pitchKI =
      configTab.add("Pitch KI", Constants.TurretConstants.PITCH_KI).getEntry();
  private final GenericEntry pitchKD =
      configTab.add("Pitch KD", Constants.TurretConstants.PITCH_KD).getEntry();
  private final DebugEntry<Double> pitchPositionEntry =
      new DebugEntry<Double>(pitchPosition, "Pitch Position", this);
  private final DebugEntry<Double> pitchGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Pitch Goal Position", this);
  private final DebugEntry<Double> pitchVelocityEntry =
      new DebugEntry<Double>(pitchVelocity, "Pitch Velocity", this);

  private final DebugEntry<Double> tagDistanceEntry =
      new DebugEntry<Double>(0.0, "LastMeasuredTagDistance", this);

  private final RobotStateManager robotStateManager;

  public TurretSubsystem(RobotStateManager robotStateManager) {
    // Initialize Motors
    turretMotor = new CANSparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
    pitchMotor = new CANSparkMax(Constants.TurretConstants.PITCH_MOTOR_ID, MotorType.kBrushless);
    pitchFeedForward =
        new ArmFeedforward(
            Constants.TurretConstants.PITCH_KS,
            Constants.TurretConstants.PITCH_KG,
            Constants.TurretConstants.PITCH_KV,
            Constants.TurretConstants.PITCH_KA);

    // Simulation
    if (Robot.isSimulation()) {
      // Turret
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
              Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY,
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

    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(Constants.TurretConstants.TURRET_SMART_CURRENT_LIMIT);

    pitchMotor.restoreFactoryDefaults();
    pitchMotor.setSmartCurrentLimit(Constants.TurretConstants.PITCH_SMART_CURRENT_LIMIT);

    this.robotStateManager = robotStateManager;

    // Soft Limits
    final double turretMinAngleRotations =
        (Constants.TurretConstants.TURRET_MIN_ANGLE_DEGREES
            / (360 * Constants.TurretConstants.TURRET_MOTOR_TURRET_RATIO));
    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, (float) turretMinAngleRotations);

    final double turretMaxAngleRotations =
        (Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES
            / (360 * Constants.TurretConstants.TURRET_CONVERSION_FACTOR));
    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward, (float) turretMaxAngleRotations);

    // Pitch
    final double pitchMinAngleRotations =
        (Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES
            / (360 * Constants.TurretConstants.PITCH_CONVERSION_FACTOR));
    pitchMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, (float) pitchMinAngleRotations);

    final double pitchMaxAngleRotations =
        (Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES
            / (360 * Constants.TurretConstants.PITCH_CONVERSION_FACTOR));
    pitchMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward, (float) pitchMaxAngleRotations);

    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    pitchMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    pitchMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    // initialze PID controller and encoder objects
    turretPIDController =
        new PIDController(
            Constants.TurretConstants.KP,
            Constants.TurretConstants.KI,
            Constants.TurretConstants.KD);

    highGearCANcoder = new CANcoder(Constants.TurretConstants.highGearCAN_CODER_ID);
    lowGearCANcoder = new CANcoder(Constants.TurretConstants.lowGearCAN_CODER_ID);
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
    turretPIDController.setPID(
        Constants.TurretConstants.TURRET_KP,
        Constants.TurretConstants.TURRET_KI,
        Constants.TurretConstants.TURRET_KD);

    simTurretEncoder =
        new SimDeviceSim(
            "CANEncoder:CANCoder (v6)", Constants.TurretConstants.highGearCAN_CODER_ID);
    simTurretPos = simTurretEncoder.getDouble("rawPositionInput");

    zeroTurret();
    turretPIDController.setIZone(Constants.TurretConstants.TURRET_KIZ);

    // Pitch
    pitchPIDController =
        new PIDController(
            Constants.TurretConstants.PITCH_KP,
            Constants.TurretConstants.PITCH_KI,
            Constants.TurretConstants.PITCH_KD);
    pitchEncoder =
        pitchMotor.getAbsoluteEncoder();

    simPitchEncoder =
        new SimDeviceSim("CANEncoder:CANCoder (v6)", Constants.TurretConstants.PITCH_ENCODER_ID);
    simPitchPos = simPitchEncoder.getDouble("rawPositionInput");

    pitchPIDController.setIZone(Constants.TurretConstants.PITCH_KIZ);

    zeroTurret();
    turretTab.add("zero turret", zeroZeroing());
  }

  private void stopTurret() {
    turretMotor.stopMotor();
  }

  public Command stowTurret() {
    return new InstantCommand(
            () -> setTurretPos(Math.toRadians(Constants.TurretConstants.TURRET_STOWED_ANGLE)))
        .alongWith(
            new InstantCommand(
                () -> setPitchPos(Math.toRadians(Constants.TurretConstants.PITCH_STOWED_ANGLE))))
        .withName("StowTurretCommand");
  }

  public Command pickup() {
    return new InstantCommand(
            () -> setTurretPos(Math.toRadians(Constants.TurretConstants.TURRET_PICKUP_ANGLE)))
        .alongWith(
            new InstantCommand(
                () -> setPitchPos(Math.toRadians(Constants.TurretConstants.PITCH_PICKUP_ANGLE))))
        .withName("StowTurretCommand");
  }

  public double calculateTurretPosition() {
    double encoderPosition =
        lowGearCANcoder.getPosition().getValueAsDouble()
            / Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO;
    return encoderPosition + Constants.TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV;
  }

  private double calculatePitchPosition() {
    return Math.toRadians(
        (pitchEncoder.getPosition() * 360)
            * Constants.TurretConstants.PITCH_CONVERSION_FACTOR);
  }

  /**
   * A command to set the current turret position as true zero.
   *
   * @return a command that sets the current position as true zero
   */
  public Command zeroZeroing() {
    return Commands.runOnce(
        () -> {
          MagnetSensorConfigs cfg = new MagnetSensorConfigs();
          cfg.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
          cfg.withMagnetOffset(0);
          CANcoderConfigurator lowGearCANcoderConfigurator = lowGearCANcoder.getConfigurator();
          CANcoderConfigurator highGearCANcoderConfigurator = highGearCANcoder.getConfigurator();
          lowGearCANcoderConfigurator.apply(cfg);
          highGearCANcoderConfigurator.apply(cfg);

          final double trueZeroLowGearOffset =
              lowGearCANcoder.getAbsolutePosition().getValueAsDouble();
          final double trueZeroHighGearOffset =
              highGearCANcoder.getAbsolutePosition().getValueAsDouble();

          final double lowGearOffset =
              trueZeroLowGearOffset
                  - TurretConstants.LOW_GEAR_CAN_CODER_RATIO
                      * TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV;
          final double highGearOffset =
              trueZeroHighGearOffset
                  - TurretConstants.HIGH_GEAR_CAN_CODER_RATIO
                      * TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV;

          MagnetSensorConfigs newCfgLowGear = new MagnetSensorConfigs();
          newCfgLowGear.withMagnetOffset(lowGearOffset);
          lowGearCANcoderConfigurator.apply(newCfgLowGear);

          MagnetSensorConfigs newCfgHighGear = new MagnetSensorConfigs();
          newCfgHighGear.withMagnetOffset(highGearOffset);
          highGearCANcoderConfigurator.apply(newCfgHighGear);

          DynamicRobotConfig dynamicConfig = new DynamicRobotConfig();
          dynamicConfig.saveTurretZero(new TurretZeroConfig(lowGearOffset, highGearOffset));
        },
        this);
  }

  /** Will calculate the current turret position and update encoders and motors off of it. */
  public void zeroTurret() {
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

  public Command zeroTurretCommand() {
    return Commands.runOnce(() -> this.zeroTurret(), this).withName("ZeroTurretCommand");
  }

  private void setTurretPos(double setpoint) {
    turretGoalPositionEntry.log(setpoint);
    turretMotor.set(
        -turretPIDController.calculate(
            turretPosition,
            MathUtil.clamp(
                setpoint,
                Math.toRadians(Constants.TurretConstants.TURRET_MIN_ANGLE_DEGREES),
                Math.toRadians(Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES))));
  }

  private void setPitchPos(double setpoint) {
    pitchGoalPositionEntry.log(setpoint);
    pitchMotor.setVoltage(
        pitchPIDController.calculate(
                pitchPosition,
                MathUtil.clamp(
                    setpoint,
                    Math.toRadians(Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES),
                    Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES)))
            + pitchFeedForward.calculate(turretPosition, 0));
  }

  private void holdPosition() {
    setTurretPos(turretPosition);
    setPitchPos(0);
  }

  public Command idleTurret() {
    return run(() -> holdPosition()).withName("idleTurret");
  }

  private void moveUp() {
    setTurretPos(turretPosition);
    setPitchPos(30);
  }

  public Command moveUpwards() {
    return run(() -> moveUp()).withName("moveShooterUp");
  }

  private void updateTurretPosition() {
    turretPosition = calculateTurretPosition();
    SmartDashboard.putNumber("Turret Position", turretPosition);
    SmartDashboard.putBoolean("Out of Bounds", Math.abs(turretPosition) > 3.14);
    SmartDashboard.putBoolean(
        "Soft limit enabled forward", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kForward));
    SmartDashboard.putBoolean(
        "Soft limit enabled reverse", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse));
  }

  private void updatePitchPosition() {
    pitchPosition = calculatePitchPosition();
    SmartDashboard.putNumber("Pitch Position", pitchPosition);
    SmartDashboard.putBoolean("Out of Bounds", Math.abs(pitchPosition) > 3.14);
    SmartDashboard.putBoolean(
        "Soft limit enabled forward", pitchMotor.isSoftLimitEnabled(SoftLimitDirection.kForward));
    SmartDashboard.putBoolean(
        "Soft limit enabled reverse", pitchMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse));
  }

  public double getTurretPos() {
    return turretPosition; // returns the absolute encoder position in radians
  }

  public double getTurretVel() {
    return turretVelocity;
  }

  public Command testTurretCommand(double degrees) {
    return runEnd(() -> setTurretPos(Math.toRadians(degrees)), this::stopTurret)
        .withName("TestTurret");
  }

  private void aimTurret() {
    double limelightTX = LimelightHelpers.getTX("limelight");
    if (limelightTX != 0
        && LimelightHelpers.getFiducialID("limelight")
            == ((robotStateManager.getAllianceColor()
                    == AllianceColor
                        .BLUE) // Default to red because that's the color on our test field
                ? Constants.TurretConstants.SPEAKER_TAG_ID_BLUE
                : Constants.TurretConstants.SPEAKER_TAG_ID_RED)) {
      // X & Rotation
      setTurretPos(Math.toRadians(limelightTX) + turretPosition);

      // Y & Pitch control
      double limelightTY = LimelightHelpers.getTY("limelight");
      double distanceToTag = tyToDistanceFromTag(limelightTY);
      tagDistanceEntry.log(distanceToTag);
      setPitchPos(distanceToShootingPitch(distanceToTag));

      if (Math.toRadians(limelightTX) + turretPosition
              > Math.toRadians(Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES)
          || Math.toRadians(limelightTX) + turretPosition
              < Math.toRadians(Constants.TurretConstants.TURRET_MIN_ANGLE_DEGREES)) {
        // TODO: Make turret rotate the drivebase if necessary and the driver thinks it's a good
        // idea
      }
    } else {
      // TODO: Make turret default to using odometry
      System.out.println("HELP");
      setTurretPos(60);
    }
  }

  private void aimTurretOdometry(Pose2d robotPos, Pose2d targetPos) {
    setTurretPos(getTurretRotationFromOdometry(robotPos, targetPos));
  }

  public Command getAimTurretCommand() {
    return run(() -> aimTurret()).withName("AimTurretCommand");
  }

  /**
   * Calculates the distance to the tag centered on the speaker from the angle that the limelight
   * sees it
   *
   * @param ty The ty output by the limelight (degrees)
   * @return The distance from the tag (meters)
   */
  private double tyToDistanceFromTag(double ty) {
    double tagTheta = Math.toRadians(ty) + Constants.TurretConstants.LIMELIGHT_PITCH_RADIANS;
    double height =
        Constants.TurretConstants.SPEAKER_TAG_CENTER_HEIGHT_METERS
            - Constants.TurretConstants.LIMELIGHT_HEIGHT_METERS;
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
    turretPIDController.setP(turretKP.getDouble(turretPosition));
    turretPIDController.setI(turretKI.getDouble(turretPosition));
    turretPIDController.setD(turretKD.getDouble(turretPosition));

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
  }

  @Override
  public void simulationPeriodic() {
    turretSim.setInput(turretMotor.get() * RobotController.getBatteryVoltage());
    turretSim.update(Robot.defaultPeriodSecs);
    turretAngleSim.setAngle(Math.toDegrees(turretSim.getAngleRads()));
    SmartDashboard.putNumber("Turret Angle", Math.toDegrees(turretSim.getAngleRads()));
    simTurretPos.set(
        Units.radiansToRotations(
            turretSim.getAngleRads() / Constants.TurretConstants.TURRET_CONVERSION_FACTOR));

    pitchSim.setInput(pitchMotor.getAppliedOutput());
    pitchSim.update(Robot.defaultPeriodSecs);
    pitchAngleSim.setAngle(Math.toDegrees(pitchSim.getAngleRads()));
    SmartDashboard.putNumber("Turret Angle", Math.toDegrees(pitchSim.getAngleRads()));
    simPitchPos.set(
        Units.radiansToRotations(
            pitchSim.getAngleRads() / Constants.TurretConstants.PITCH_CONVERSION_FACTOR));
  }

  private double getTurretRotationFromOdometry(Pose2d robotPos, Pose2d targetPos) {
    return Math.atan2(robotPos.getY() - targetPos.getY(), robotPos.getX() - targetPos.getX())
        + robotPos.getRotation().getRadians();
  }
}
