// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.LimelightHelpers;
import java.util.function.Consumer;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax turretMotor;
  private SingleJointedArmSim turretSim;
  private Mechanism2d turretMech;
  private MechanismRoot2d root;
  private MechanismLigament2d turretAngleSim;
  private ShuffleboardTab turretTab = Shuffleboard.getTab("Turret Tab");
  private SimDeviceSim simEncoder;
  private SimDouble simTurretPos;

  private PIDController turretPIDController;
  private CANcoder m_encoder;
  private double turretPosition;
  private double turretVelocity;
  private Consumer<Double> testPosition;
  private DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Position", this);
  private DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Velocity", this);

  private final RobotStateManager robotStateManager;

  public TurretSubsystem(RobotStateManager robotStateManager) {
    turretMotor = new CANSparkMax(Constants.TurretConstants.MOTOR_ID, MotorType.kBrushless);

    // Simulation
    if (Robot.isSimulation()) {
      turretSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              4,
              3.5 * 0.1016 * 0.1016 / 3,
              0.1016,
              -Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES),
              Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES),
              false,
              0);
    }
    turretMech = new Mechanism2d(4, 4);
    root = turretMech.getRoot("Root", 2, 2);
    turretAngleSim =
        root.append(new MechanismLigament2d("Turret", 2, 0, 5, new Color8Bit(Color.kRed)));
    turretTab.add("Turret", turretMech);

    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(40);

    this.robotStateManager = robotStateManager;

    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -2);
    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 2);

    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    // initialze PID controller and encoder objects
    turretPIDController =
        new PIDController(
            Constants.TurretConstants.KP,
            Constants.TurretConstants.KI,
            Constants.TurretConstants.KD);
    m_encoder = new CANcoder(Constants.TurretConstants.CANcoder_ID);

    simEncoder =
        new SimDeviceSim("CANEncoder:CANCoder (v6)", Constants.TurretConstants.CANcoder_ID);
    simTurretPos = simEncoder.getDouble("rawPositionInput");

    zeroTurretEncoder();
    turretPIDController.setIZone(Constants.TurretConstants.KIZ);
  }

  public void stopTurret() {
    turretMotor.stopMotor();
  }

  public Command stowTurret() {
    return new InstantCommand(() -> setTurretPos(Math.toRadians(0))).withName("StowTurretCommand");
  }

  public void setTurretPos(double setpoint) {
    turretMotor.set(
        -turretPIDController.calculate(
            turretPosition,
            MathUtil.clamp(
                setpoint,
                Math.toRadians(-Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES),
                Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES))));
  }

  public void holdPosition() {
    setTurretPos(turretPosition);
  }

  public void zeroTurretEncoder() {
    m_encoder.setPosition(0.0);
  }

  public void updateTurretPosition() {
    turretPosition =
        Math.toRadians(
            ((m_encoder.getPosition().getValueAsDouble()) * 360)
                * Constants.TurretConstants.CONVERSION_FACTOR);
    SmartDashboard.putNumber("Turret Position", turretPosition);
    SmartDashboard.putBoolean("Out of Bounds", Math.abs(turretPosition) > 3.14);
    SmartDashboard.putBoolean(
        "Soft limit enabled forward", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kForward));
    SmartDashboard.putBoolean(
        "Soft limit enabled reverse", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse));
  }

  public void lockOntoTag(double rotationFromTag) {
    turretMotor.set(turretPIDController.calculate(rotationFromTag, 0.0));
  }

  public double getTurretPos() {
    return turretPosition; // returns the absolute encoder position in radians
  }

  public double getTurretVel() {
    return turretVelocity;
  }

  public Command testTurretCommand() {
    return runEnd(() -> setTurretPos(Math.toRadians(-60)), this::stopTurret);
  }

  public void LockTurret() {
    lockOntoTag(Math.toRadians(LimelightHelpers.getTX("")));
  }

  public void TurretOdomCommand(Pose2d robotPos, Pose2d targetPos) {
    setTurretPos(getTurretRotationFromOdometry(robotPos, targetPos));
  }

  public Command buildTurretCommand(boolean limelightVisible, Pose2d robotPos, Pose2d targetPos) {
    return limelightVisible
        ? run(this::LockTurret)
        : runEnd(() -> TurretOdomCommand(robotPos, targetPos), this::stopTurret);
  }

  @Override
  public void periodic() {
    updateTurretPosition();
    turretVelocity =
        (m_encoder.getVelocity().getValueAsDouble())
            * 60; // changing from rotations per second to rotations per minute or rpm
    turretPositionEntry.log(turretPosition);
    turretVelocityEntry.log(turretVelocity);
  }

  @Override
  public void simulationPeriodic() {
    // System.out.println(Math.toDegrees(turretMotor.get()));
    turretSim.setInput(turretMotor.get() * RobotController.getBatteryVoltage());
    turretSim.update(Robot.defaultPeriodSecs);
    turretAngleSim.setAngle(Math.toDegrees(turretSim.getAngleRads()));
    SmartDashboard.putNumber("Turret Angle", Math.toDegrees(turretSim.getAngleRads()));
    simTurretPos.set(
        Units.radiansToRotations(
            turretSim.getAngleRads() / Constants.TurretConstants.CONVERSION_FACTOR));
    // turretMotor.set(1);
  }

  public double getTurretRotationFromOdometry(
      Pose2d robotPos, Pose2d targetPos) { // Doesn't work right now but will be used later
    return Math.atan2(robotPos.getY() - targetPos.getY(), robotPos.getX() - targetPos.getX())
        + robotPos.getRotation().getRadians();
  }
}
