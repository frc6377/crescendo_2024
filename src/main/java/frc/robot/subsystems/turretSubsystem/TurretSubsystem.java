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
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.enabledSubsystems;
import frc.robot.Robot;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.config.TurretZeroConfig;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.HowdyMath;

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
  private double pitchPosition;
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

  private final ShuffleboardTab turretTab = Shuffleboard.getTab(this.getName());
  private final DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Turret Position", this);
  private final DebugEntry<Double> turretGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Turret Goal Position", this);
  private final DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Turret Velocity", this);
  private final DebugEntry<Double> pitchPositionEntry =
      new DebugEntry<Double>(pitchPosition, "Pitch Position", this);
  private final DebugEntry<Double> pitchGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Pitch Goal Position", this);
  private final DebugEntry<Double> pitchVelocityEntry =
      new DebugEntry<Double>(pitchVelocity, "Pitch Velocity", this);
  private final DebugEntry<Double> motorOutputEntry =
      new DebugEntry<Double>(0.0, "Turret Motor Output", this);

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
    } else {
      simulationInit();
    }

    this.robotStateManager = robotStateManager;
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void periodic() {
    updateTurretPosition();
    updatePitchPosition();
    updateTelemetry();

    motorOutputEntry.log(turretMotor.get());
  }

  private void updateTurretPosition() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return;
    turretPosition = getTurretRotation();
    turretMotor.set(turretPIDController.calculate(turretPosition));
  }

  private void updatePitchPosition() {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return;
    pitchPosition = getTurretPitch();
    pitchMotor.set(
        pitchPIDController.calculate(pitchPosition)
            + pitchFeedForward.calculate(turretPosition, 0));
  }

  // ----------------------------- Setters -----------------------------

  public void setTurretPos(double setpoint) {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return;
    turretGoalPositionEntry.log(setpoint);
    turretPIDController.setSetpoint(
        MathUtil.clamp(
            setpoint,
            Math.toRadians(Constants.TurretConstants.TURRET_MIN_ANGLE_DEGREES),
            Math.toRadians(Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES)));
  }

  public void setPitchPos(double setpoint) {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return;
    pitchGoalPositionEntry.log(setpoint);
    pitchPIDController.setSetpoint(
        MathUtil.clamp(
            setpoint,
            Math.toRadians(Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES),
            Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES)));
  }

  public void stopTurret() {
    if (enabledSubsystems.turretPitchEnabled) pitchMotor.stopMotor();
    if (enabledSubsystems.turretRotationEnabled) turretMotor.stopMotor();
  }

  // ----------------------------- Getters -----------------------------

  protected VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
  }

  /**
   * Returns the turret rotation in radians.
   *
   * @return the turret rotation in radians.
   */
  public double getTurretRotation() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    double encoderPosition =
        lowGearCANcoder.getPosition().getValueAsDouble()
            / Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO;
    return (encoderPosition + Constants.TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV)
        * Math.PI
        * 2;
  }

  /**
   * Returns the turret pitch in radians.
   *
   * @return the turret pitch in radians.
   */
  private double getTurretPitch() {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return 0;
    return Units.rotationsToRadians(
        pitchEncoder.getPosition() * Constants.TurretConstants.PITCH_CONVERSION_FACTOR);
  }

  public double getTurretPos() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    return turretPosition; // returns the absolute encoder position in radians
  }

  public double getTurretVel() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    return turretVelocity;
  }

  protected CANcoder getLowGearCANCoder() {
    return lowGearCANcoder;
  }

  protected CANcoder getHighGearCANCoder() {
    return highGearCANcoder;
  }

  // ----------------------------- Zeroing -----------------------------

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

  // ----------------------------- Simulation -----------------------------

  public void simulationInit() {
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

  private void updateTelemetry() {
    if (enabledSubsystems.turretRotationEnabled) {
      turretVelocity =
          (lowGearCANcoder.getVelocity().getValueAsDouble()
                  * Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO)
              * 60; // changing from rotations per second to rotations per minute or rpm
      turretPositionEntry.log(turretPosition);
      turretVelocityEntry.log(turretVelocity);
    }
    if (enabledSubsystems.turretPitchEnabled) {
      pitchVelocity =
          pitchEncoder.getVelocity()
              * 60; // changing from rotations per second to rotations per minute or rpm
      pitchPositionEntry.log(pitchPosition);
      pitchVelocityEntry.log(pitchVelocity);
    }
  }

  public void requestRotationOutput(double calculate) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'requestRotationOutput'");
  }
}
