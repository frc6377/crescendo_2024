// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.LimelightHelpers;
import java.util.function.Consumer;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax turretMotor;
  private SingleJointedArmSim turretSim;
  private Mechanism2d turretMech;
  private MechanismRoot2d turretRoot;
  private MechanismLigament2d turretAngleSim;
  private ShuffleboardTab turretTab = Shuffleboard.getTab("Turret Tab");
  private SimDeviceSim simTurretEncoder;
  private SimDouble simTurretPos;

  private PIDController turretPIDController;
  private CANcoder m_turretEncoder;
  private double turretPosition;
  private double turretVelocity;
  private Consumer<Double> turretTestPosition;
  private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
  private GenericEntry turretKP =
      configTab.add("Turret KP", Constants.TurretConstants.TURRET_KP).getEntry();
  private GenericEntry turretKI =
      configTab.add("Turret KI", Constants.TurretConstants.TURRET_KI).getEntry();
  private GenericEntry turretKD =
      configTab.add("Turret KD", Constants.TurretConstants.TURRET_KD).getEntry();
  private DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Position", this);
  private DebugEntry<Double> turretGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Goal Position", this);
  private DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Velocity", this);

  private CANSparkMax pitchMotor;
  private SingleJointedArmSim pitchSim;
  private Mechanism2d pitchMech;
  private MechanismRoot2d pitchRoot;
  private MechanismLigament2d pitchAngleSim;
  private SimDeviceSim simPitchEncoder;
  private SimDouble simPitchPos;

  private PIDController pitchPIDController;
  private CANcoder m_pitchEncoder;
  private double pitchPosition;
  private double pitchVelocity;
  private Consumer<Double> pitchTestPosition;
  private GenericEntry pitchKP =
      configTab.add("Pitch KP", Constants.TurretConstants.TURRET_KP).getEntry();
  private GenericEntry pitchKI =
      configTab.add("Pitch KI", Constants.TurretConstants.TURRET_KI).getEntry();
  private GenericEntry pitchKD =
      configTab.add("Pitch KD", Constants.TurretConstants.TURRET_KD).getEntry();
  private DebugEntry<Double> pitchPositionEntry =
      new DebugEntry<Double>(pitchPosition, "Position", this);
  private DebugEntry<Double> pitchGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Goal Position", this);
  private DebugEntry<Double> pitchVelocityEntry =
      new DebugEntry<Double>(pitchVelocity, "Velocity", this);

  private DebugEntry<Double> tagDistanceEntry =
      new DebugEntry<Double>(0.0, "LastMeasuredTagDistance", this);

  private final RobotStateManager robotStateManager;

  public TurretSubsystem(RobotStateManager robotStateManager) {
    turretMotor = new CANSparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
    pitchMotor = new CANSparkMax(Constants.TurretConstants.PITCH_MOTOR_ID, MotorType.kBrushless);

    // Simulation
    if (Robot.isSimulation()) {
      // Turret
      turretSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              4,
              3.5 * 0.1016 * 0.1016 / 3,
              0.1016,
              -Math.toRadians(Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES),
              Math.toRadians(Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES),
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
              4,
              3.5 * 0.1016 * 0.1016 / 3,
              0.1016,
              -Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES),
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
    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse,
        (float)
            (-Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES
                / (360 * Constants.TurretConstants.TURRET_CONVERSION_FACTOR)));
    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward,
        (float)
            (Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES
                / (360 * Constants.TurretConstants.TURRET_CONVERSION_FACTOR)));

    pitchMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse,
        (float)
            (Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES
                / (360 * Constants.TurretConstants.PITCH_CONVERSION_FACTOR)));
    pitchMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward,
        (float)
            (Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES
                / (360 * Constants.TurretConstants.PITCH_CONVERSION_FACTOR)));

    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    pitchMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    pitchMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    // initialze PID controller and encoder objects
    turretPIDController =
        new PIDController(
            Constants.TurretConstants.TURRET_KP,
            Constants.TurretConstants.TURRET_KI,
            Constants.TurretConstants.TURRET_KD);
    m_turretEncoder = new CANcoder(Constants.TurretConstants.TURRET_CANcoder_ID);

    simTurretEncoder =
        new SimDeviceSim("CANEncoder:CANCoder (v6)", Constants.TurretConstants.TURRET_CANcoder_ID);
    simTurretPos = simTurretEncoder.getDouble("rawPositionInput");

    zeroTurretEncoder();
    turretPIDController.setIZone(Constants.TurretConstants.TURRET_KIZ);

    // Pitch
    pitchPIDController =
        new PIDController(
            Constants.TurretConstants.PITCH_KP,
            Constants.TurretConstants.PITCH_KI,
            Constants.TurretConstants.PITCH_KD);
    m_pitchEncoder = new CANcoder(Constants.TurretConstants.PITCH_CANcoder_ID);

    simPitchEncoder =
        new SimDeviceSim("CANEncoder:CANCoder (v6)", Constants.TurretConstants.PITCH_CANcoder_ID);
    simPitchPos = simPitchEncoder.getDouble("rawPositionInput");

    zeroTurretEncoder();
    pitchPIDController.setIZone(Constants.TurretConstants.PITCH_KIZ);
  }

  private void stopTurret() {
    turretMotor.stopMotor();
  }

  public Command stowTurret() {
    return new InstantCommand(() -> setTurretPos(Math.toRadians(0)))
        .alongWith(
            new InstantCommand(
                () ->
                    setPitchPos(
                        Math.toRadians(0 /*TODO: If the stowed positon isn't 0, make it so.*/))))
        .withName("StowTurretCommand");
  }

  private void setTurretPos(double setpoint) {
    turretGoalPositionEntry.log(setpoint);
    turretMotor.set(
        -turretPIDController.calculate(
            turretPosition,
            MathUtil.clamp(
                setpoint,
                Math.toRadians(-Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES),
                Math.toRadians(Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES))));
  }

  private void setPitchPos(double setpoint) {
    pitchGoalPositionEntry.log(setpoint);
    pitchMotor.set(
        pitchPIDController.calculate(
                pitchPosition,
                MathUtil.clamp(
                    setpoint,
                    Math.toRadians(-Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES),
                    Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES)))
            + calculateArbitraryFeedForward(pitchPosition));
  }

  private void holdPosition() {
    setTurretPos(turretPosition);
    setPitchPos(pitchPosition);
  }

  public Command idleTurret() {
    return run(() -> holdPosition()).withName("idleTurret");
  }

  private void moveUp() {
    setTurretPos(turretPosition);
    setPitchPos(pitchPosition + 0.5);
  }

  public Command moveUpwards() {
    return run(() -> moveUp()).withName("moveShooterUp");
  }

  private void zeroTurretEncoder() {
    m_turretEncoder.setPosition(0.0);
  }

  private void updateTurretPosition() {
    turretPosition =
        Math.toRadians(
            ((m_turretEncoder.getPosition().getValueAsDouble()) * 360)
                * Constants.TurretConstants.TURRET_CONVERSION_FACTOR);
    SmartDashboard.putNumber("Turret Position", turretPosition);
  }

  private void updatepitchPosition() {
    pitchPosition =
        Math.toRadians(
            ((m_pitchEncoder.getPosition().getValueAsDouble()) * 360)
                * Constants.TurretConstants.PITCH_CONVERSION_FACTOR);
    SmartDashboard.putNumber("Pitch Position", pitchPosition);
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

      if (Math.abs(Math.toRadians(limelightTX) + turretPosition)
          > Math.toRadians(Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES)) {
        // TODO: Make turret rotate the drivebase if necessary and the driver thinks it's a good
        // idea
      }
    } else {
      // TODO: Make turret default to using odometry
      setTurretPos(turretPosition);
    }
  }

  private void aimTurretOdometry(Pose2d robotPos, Pose2d targetPos) {
    setTurretPos(getTurretRotationFromOdometry(robotPos, targetPos));
  }

  public Command getAimTurretCommand() {
    return run(() -> aimTurret()).withName("AimTurretCommand");
  }

  private double tyToDistanceFromTag(double ty) {
    double tagTheta = Math.toRadians(ty) + Constants.TurretConstants.LIMELIGHT_PITCH_RADIANS;
    double height =
        Constants.TurretConstants.SPEAKER_TAG_CENTER_HEIGHT_INCHES
            - Constants.TurretConstants.LIMELIGHT_HEIGHT_INCHES;
    double distance = height / Math.tan(tagTheta);
    return distance;
  }

  private double distanceToShootingPitch(double distance) {
    return distance; // TODO: Make and use a real formula(use testing, not physics)
  }

  private double calculateArbitraryFeedForward(double angle) {
    double tourque = Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY * Math.cos(angle);
    return tourque
        * Constants.TurretConstants.SHOOTER_MASS
        * 9.8
        * Constants.TurretConstants.PITCH_NEWTONS_TO_MOTOR_POWER;
  }

  @Override
  public void periodic() {
    turretPIDController.setP(turretKP.getDouble(turretPosition));
    turretPIDController.setI(turretKI.getDouble(turretPosition));
    turretPIDController.setD(turretKD.getDouble(turretPosition));
    updateTurretPosition();
    turretVelocity =
        (m_turretEncoder.getVelocity().getValueAsDouble())
            * 60; // changing from rotations per second to rotations per minute or rpm
    turretPositionEntry.log(turretPosition);
    turretVelocityEntry.log(turretVelocity);
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

    pitchSim.setInput(pitchMotor.get() * RobotController.getBatteryVoltage());
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
