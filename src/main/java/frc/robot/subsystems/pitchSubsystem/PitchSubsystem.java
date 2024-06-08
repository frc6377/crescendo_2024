// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pitchSubsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.DebugEntry;

public class PitchSubsystem extends SubsystemBase {
  private final RobotStateManager RSM;

  private final CANSparkMaxSim pitchMotor;
  private final PIDController pitchPIDController;
  private final ArmFeedforward pitchFeedForward;

  private final SparkAbsoluteEncoder pitchEncoder;
  private final AbsoluteEncoder pitchAbsoluteEncoder;

  private double pitchPosition = 10;
  private double pitchVelocity;

  private SingleJointedArmSim pitchSim;
  private Mechanism2d pitchMech;
  private MechanismRoot2d pitchRoot;
  private MechanismLigament2d pitchAngleSim;

  private final ShuffleboardTab pitchTab = Shuffleboard.getTab(this.getName());
  private final DebugEntry<Double> pitchPositionEntry =
      new DebugEntry<Double>(pitchPosition, "Pitch Position (Degrees)", this);
  private final DebugEntry<Double> pitchGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Pitch Goal Position (Degrees)", this);
  private final DebugEntry<Double> pitchVelocityEntry =
      new DebugEntry<Double>(pitchVelocity, "Pitch Velocity (RPM)", this);
  private final DebugEntry<Double> pitchCurrentLog =
      new DebugEntry<Double>(0d, "Pitch Current (Amps)", this);
  private DebugEntry<Double> pitchMotorOutput =
      new DebugEntry<Double>(0.0, "Pitch Motor Output", this);

  private boolean usingPitchPid = true;

  public PitchSubsystem(RobotStateManager robotStateManager, VisionSubsystem visionSubsystem) {
    RSM = robotStateManager;

    pitchMotor = new CANSparkMaxSim(Constants.TurretConstants.PITCH_MOTOR_ID, MotorType.kBrushless);
    pitchMotor.restoreFactoryDefaults();
    pitchMotor.setSmartCurrentLimit(Constants.TurretConstants.PITCH_SMART_CURRENT_LIMIT);
    pitchMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    pitchAbsoluteEncoder = pitchMotor.getAbsoluteEncoder();
    pitchAbsoluteEncoder.setPositionConversionFactor(1);
    pitchAbsoluteEncoder.setZeroOffset(TurretConstants.PITCH_ZERO_OFFSET);
    pitchAbsoluteEncoder.setInverted(true);
    pitchMotor.setIdleMode(IdleMode.kCoast);

    pitchPIDController = TurretConstants.PITCH_PID.getPIDController();
    TurretConstants.PITCH_PID.createTunableNumbers("Pitch Motor", pitchPIDController, this);
    pitchFeedForward = TurretConstants.PITCH_FF.getArmFeedforward();

    pitchEncoder = pitchMotor.getAbsoluteEncoder();

    // pitch
    if (Robot.isSimulation()) {
      pitchSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              Constants.TurretConstants.PITCH_CONVERSION_FACTOR,
              Constants.TurretConstants.SHOOTER_MASS
                  * Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY
                  * Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY
                  / 3,
              Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY
                  / TurretConstants.SIMULATION_CG_MAGIC_NUMBER,
              Math.toRadians(Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES),
              Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES),
              true,
              0);
      pitchMech = new Mechanism2d(4, 4);
      pitchRoot = pitchMech.getRoot("Root", 2, 2);
      pitchAngleSim =
          pitchRoot.append(new MechanismLigament2d("Pitch", 2, 0, 5, new Color8Bit(Color.kBlue)));
      pitchTab.add("Pitch", pitchMech);
    }
  }

  public void stopPitch() {
    pitchMotor.stopMotor();
  }

  private double calculatePitchPosition() {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return 0;
    double pos = pitchEncoder.getPosition();
    if (pos > 0.5) {
      pos -= 1;
    }
    return Units.rotationsToRadians(pos);
  }

  public void setPitchPos(double setpoint) {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return;
    usingPitchPid = true;
    pitchPIDController.setSetpoint(
        MathUtil.clamp(
            setpoint,
            Math.toRadians(Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES),
            Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES)));
    pitchGoalPositionEntry.log(Math.toDegrees(setpoint));
  }

  public void holdPosition() {
    if (Constants.enabledSubsystems.turretPitchEnabled) {
      usingPitchPid = false;
      pitchMotor.setVoltage(-0.1);
    }
  }

  private void updatePitchPosition() {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return;
    pitchPosition = calculatePitchPosition();
  }

  /**
   * The elevation of the turret in radians
   *
   * @return the current elevation of the turret
   */
  public double getPitch() {
    return pitchPosition;
  }

  DebugEntry<Boolean> atPitchLog = new DebugEntry<Boolean>(false, "Pitch as setpoint", this);

  public boolean pitchAtSetpoint() {
    pitchPIDController.setTolerance(TurretConstants.PITCH_TOLERANCE_RADIANS);
    boolean result = pitchPIDController.atSetpoint();
    atPitchLog.log(result);
    return result;
  }

  @Override
  public void periodic() {
    if (enabledSubsystems.turretPitchEnabled) {
      updatePitchPosition();
      if (usingPitchPid) {
        pitchMotor.setVoltage(
            pitchPIDController.calculate(pitchPosition)
                + pitchFeedForward.calculate(pitchPosition, 0));
      }
      pitchVelocity =
          pitchEncoder.getVelocity()
              * 60; // changing from rotations per second to rotations per minute or rpm
      pitchPositionEntry.log(Units.radiansToDegrees(pitchPosition));
      pitchVelocityEntry.log(pitchVelocity);
      pitchCurrentLog.log(pitchMotor.getOutputCurrent());
      pitchMotorOutput.log(pitchMotor.getAppliedOutput());
    }
  }

  @Override
  public void simulationPeriodic() {
    pitchSim.setInput(pitchMotor.getAppliedOutput());
    pitchSim.update(Robot.defaultPeriodSecs);
    pitchAngleSim.setAngle(Math.toDegrees(pitchSim.getAngleRads()));
    pitchMotor.setAbsolutePosition(Units.radiansToRotations(pitchSim.getAngleRads()));
  }
}
