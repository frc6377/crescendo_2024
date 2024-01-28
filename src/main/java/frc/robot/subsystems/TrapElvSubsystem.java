// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapElvConstants;
import frc.robot.Robot;
import frc.robot.networktables.DebugEntry;
import java.util.function.BooleanSupplier;

public class TrapElvSubsystem extends SubsystemBase {
  // Wrist motors
  private final CANSparkMaxSim wristMotor;
  private final CANSparkMax rollerMotor;

  // Elevator motors
  private final CANSparkMaxSim baseMotor1;
  private final CANSparkMax baseMotor2;
  private final CANSparkMaxSim scoringMotor;

  // Beam Breaks
  private final DigitalInput sourceBreak;
  private final DigitalInput groundBreak;

  // Limit Switches
  private final DigitalInput baseLimit;
  private final DigitalInput scoringLimit;

  // Encoders
  private final CANcoder wristEncoder;

  private DebugEntry<Boolean> sourceLog;
  private DebugEntry<Boolean> groundLog;
  private DebugEntry<Boolean> baseLog;
  private DebugEntry<Boolean> scoringLog;

  private Mechanism2d elvMechanism;
  private MechanismRoot2d root;
  private MechanismLigament2d baseMech;
  private MechanismLigament2d scoringMech;
  private MechanismLigament2d wristMech;

  private ElevatorSim m_baseElevatorSim;
  private ElevatorSim m_scoringElevatorSim;
  private SingleJointedArmSim m_wristMotorSim;

  private ShuffleboardTab TrapElvTab = Shuffleboard.getTab("Trap Arm Tab");
  private GenericEntry baseGoal = TrapElvTab.add("Base Goal", 0).getEntry();

  // States
  public static enum TrapElvState {
    // Degrees, elv height, elv height
    STOWED(0.0, 0.0, 0.0),
    FROM_INTAKE(15.0, 0.0, 0.0),
    FROM_SOURCE(150.0, 0.0, 12.0),
    TRAP_SCORE(0.0, 12.0, 12.0),
    AMP_SCORE(90.0, 0.0, 12.0);

    private double wristPose;
    private double basePose;
    private double scoringPose;

    TrapElvState(double wrist, double base, double scoring) {
      this.wristPose = wrist;
      this.basePose = base;
      this.scoringPose = scoring;
    }

    private double feetToRotations(double f) {
      return Units.inchesToMeters(f)
          / (2 * Math.PI * Units.inchesToMeters(1))
          * TrapElvConstants.ELV_GEAR_RATIO;
    }

    public Double getWristPose() {
      return Units.degreesToRotations(wristPose);
    }

    public Double getBasePose() {
      return feetToRotations(basePose);
    }

    public Double getScoringPose() {
      return feetToRotations(scoringPose);
    }
  }

  /** Creates a new TrapArm. */
  public TrapElvSubsystem() {
    // Wrist
    wristMotor = new CANSparkMaxSim(TrapElvConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    wristMotor.getPIDController().setP(TrapElvConstants.WRIST_PID[0]);
    wristMotor.getPIDController().setI(TrapElvConstants.WRIST_PID[1]);
    wristMotor.getPIDController().setD(TrapElvConstants.WRIST_PID[2]);
    wristMotor.getPIDController().setIZone(TrapElvConstants.WRIST_PID[3]);
    wristMotor.getPIDController().setFF(TrapElvConstants.WRIST_PID[4]);
    TrapElvTab.add("Wrist PID", wristMotor.getPIDController());

    wristEncoder = new CANcoder(6);

    rollerMotor = new CANSparkMax(TrapElvConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    rollerMotor.restoreFactoryDefaults();

    sourceBreak = new DigitalInput(TrapElvConstants.SOURCE_BREAK_ID);
    groundBreak = new DigitalInput(TrapElvConstants.GROUND_BREAK_ID);

    // Elv

    baseMotor1 = new CANSparkMaxSim(TrapElvConstants.BASE_MOTOR1_ID, MotorType.kBrushless);
    baseMotor1.restoreFactoryDefaults();
    baseMotor1.getPIDController().setP(TrapElvConstants.BASE_PID[0]);
    baseMotor1.getPIDController().setI(TrapElvConstants.BASE_PID[1]);
    baseMotor1.getPIDController().setD(TrapElvConstants.BASE_PID[2]);
    baseMotor1.getPIDController().setIZone(TrapElvConstants.BASE_PID[3]);
    baseMotor1.getPIDController().setFF(TrapElvConstants.BASE_PID[4]);
    TrapElvTab.add("Base Elv PID", baseMotor1.getPIDController());

    baseMotor2 = new CANSparkMax(TrapElvConstants.BASE_MOTOR2_ID, MotorType.kBrushless);
    baseMotor2.restoreFactoryDefaults();
    baseMotor2.getPIDController().setP(TrapElvConstants.BASE_PID[0]);
    baseMotor2.getPIDController().setI(TrapElvConstants.BASE_PID[1]);
    baseMotor2.getPIDController().setD(TrapElvConstants.BASE_PID[2]);
    baseMotor2.getPIDController().setIZone(TrapElvConstants.BASE_PID[3]);
    baseMotor2.getPIDController().setFF(TrapElvConstants.BASE_PID[4]);

    baseLimit = new DigitalInput(TrapElvConstants.BASE_BREAK_ID);

    scoringMotor = new CANSparkMaxSim(TrapElvConstants.SCORING_MOTOR_ID, MotorType.kBrushless);
    scoringMotor.restoreFactoryDefaults();
    scoringMotor.getPIDController().setP(TrapElvConstants.SCORING_PID[0]);
    scoringMotor.getPIDController().setI(TrapElvConstants.SCORING_PID[1]);
    scoringMotor.getPIDController().setD(TrapElvConstants.SCORING_PID[2]);
    scoringMotor.getPIDController().setIZone(TrapElvConstants.SCORING_PID[3]);
    scoringMotor.getPIDController().setFF(TrapElvConstants.SCORING_PID[4]);
    TrapElvTab.add("Scoring Elv PID", scoringMotor.getPIDController());

    scoringLimit = new DigitalInput(TrapElvConstants.SCORING_BREAK_ID);

    // SmartDashboard
    sourceLog = new DebugEntry<Boolean>(baseLimit.get(), "Source Beam Break", this);
    groundLog = new DebugEntry<Boolean>(baseLimit.get(), "Ground Beam Break", this);
    baseLog = new DebugEntry<Boolean>(baseLimit.get(), "Base Limit Switch", this);
    scoringLog = new DebugEntry<Boolean>(baseLimit.get(), "Scoring Limit Switch", this);

    // Simulation
    if (Robot.isSimulation()) {
      // 2D Mechanism
      elvMechanism = new Mechanism2d(2, 2);
      root = elvMechanism.getRoot("Root", 1, 0);
      baseMech =
          root.append(new MechanismLigament2d("Base Elv", 0, 90, 20, new Color8Bit(Color.kBlue)));
      scoringMech =
          baseMech.append(
              new MechanismLigament2d("Scoring Elv", 0, 0, 15, new Color8Bit(Color.kAqua)));
      wristMech =
          scoringMech.append(
              new MechanismLigament2d(
                  "Wrist Mech", TrapElvConstants.WRIST_LENGTH, 150, 10, new Color8Bit(Color.kRed)));

      m_baseElevatorSim =
          new ElevatorSim(
              DCMotor.getNEO(1),
              TrapElvConstants.ELV_GEAR_RATIO,
              TrapElvConstants.ELV_LIFT_MASS,
              Units.inchesToMeters(1),
              TrapElvConstants.ELV_MIN_HEIGHT,
              TrapElvConstants.ELV_MAX_HEIGHT,
              true,
              0);

      m_scoringElevatorSim =
          new ElevatorSim(
              DCMotor.getNEO(1),
              TrapElvConstants.ELV_GEAR_RATIO,
              TrapElvConstants.ELV_LIFT_MASS,
              Units.inchesToMeters(1),
              TrapElvConstants.ELV_MIN_HEIGHT,
              TrapElvConstants.ELV_MAX_HEIGHT,
              true,
              0);

      m_wristMotorSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              TrapElvConstants.ELV_GEAR_RATIO,
              TrapElvConstants.ELV_LIFT_MASS * Math.pow(TrapElvConstants.WRIST_LENGTH, 2.0),
              TrapElvConstants.WRIST_LENGTH,
              TrapElvConstants.WRIST_MIN_ANGLE, // min rotation
              TrapElvConstants.WRIST_MAX_ANGLE, // max rotation
              true,
              TrapElvConstants.WRIST_MIN_ANGLE);

      TrapElvTab.add("Trap Arm Mech", elvMechanism);
    }
  }

  // Boolean Suppliers
  public BooleanSupplier getSourceBreak() {
    return () -> sourceBreak.get();
  }

  public BooleanSupplier getGroundBreak() {
    return () -> groundBreak.get();
  }

  public BooleanSupplier getBaseLimit() {
    return () -> baseLimit.get();
  }

  public BooleanSupplier getScoringLimit() {
    return () -> scoringLimit.get();
  }

  // Commands
  public Command setRoller(double s) {
    return run(
        () -> {
          rollerMotor.set(s);
        });
  }

  public Command stopRoller() {
    return run(
        () -> {
          rollerMotor.stopMotor();
        });
  }

  public Command intakeSource() {
    return startEnd(
        () -> {
          setTrapArm(TrapElvState.FROM_SOURCE);
          rollerMotor.set(TrapElvConstants.ROLLER_INTAKE_SPEED);
        },
        () -> {
          stowTrapElv();
        });
  }

  public Command intakeGround() {
    return startEnd(
        () -> {
          setTrapArm(TrapElvState.FROM_SOURCE);
          rollerMotor.set(TrapElvConstants.ROLLER_INTAKE_SPEED);
        },
        () -> {
          stowTrapElv();
        });
  }

  public Command scoreAMP() {
    return startEnd(
        () -> {
          setTrapArm(TrapElvState.AMP_SCORE);
          setRoller(-TrapElvConstants.ROLLER_SCORING_SPEED);
        },
        () -> {
          stowTrapElv();
        });
  }

  public Command scoreTrap() {
    return startEnd(
        () -> {
          setTrapArm(TrapElvState.TRAP_SCORE);
          rollerMotor.set(TrapElvConstants.ROLLER_INTAKE_SPEED);
        },
        () -> {
          stowTrapElv();
        });
  }

  public void stowTrapElv() {
    setTrapArm(TrapElvState.STOWED);
    rollerMotor.stopMotor();
  }

  public Command zeroArm() {
    return startEnd(
        () -> {
          // Command for zeroing elevator if elevator happens to be not at zero
          // Runs elevator motors until there limit switches are pressed
          if (!baseLimit.get()) {
            baseMotor1.set(TrapElvConstants.ELV_ZEROING_SPEED);
            baseMotor2.set(TrapElvConstants.ELV_ZEROING_SPEED);
          } else {
            baseMotor1.stopMotor();
            baseMotor2.stopMotor();
          }
          if (!scoringLimit.get()) {
            scoringMotor.set(TrapElvConstants.ELV_ZEROING_SPEED);
          } else {
            scoringMotor.stopMotor();
          }
        },
        () -> {
          baseMotor1.stopMotor();
          baseMotor2.stopMotor();
          scoringMotor.stopMotor();
        });
  }

  public void setTrapArm(TrapElvState state) {
    baseGoal.setDouble(Units.metersToInches(TrapElvConstants.ELV_MIN_HEIGHT) + state.basePose);
    SmartDashboard.putNumber("Wrist Goal", state.wristPose - TrapElvConstants.WRIST_MIN_ANGLE);
    wristMotor.getPIDController().setReference(state.getWristPose(), ControlType.kPosition);
    baseMotor1.getPIDController().setReference(state.getBasePose(), ControlType.kPosition);
    baseMotor2.getPIDController().setReference(state.getBasePose(), ControlType.kPosition);
    scoringMotor.getPIDController().setReference(state.getScoringPose(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    sourceLog.log(sourceBreak.get());
    groundLog.log(groundBreak.get());
    baseLog.log(baseLimit.get());
    scoringLog.log(sourceBreak.get());
  }

  @Override
  public void simulationPeriodic() {
    for (double i = 0; i < Robot.defaultPeriodSecs; i += CANSparkMaxSim.kPeriod) {
      m_baseElevatorSim.setInput(baseMotor1.get() * RobotController.getBatteryVoltage());
      m_baseElevatorSim.update(CANSparkMaxSim.kPeriod);
      baseMotor1.update(
          m_baseElevatorSim.getVelocityMetersPerSecond()
              * TrapElvConstants.ELV_GEAR_RATIO
              / Units.inchesToMeters(1));

      m_scoringElevatorSim.setInput(scoringMotor.get() * RobotController.getBatteryVoltage());
      m_scoringElevatorSim.update(CANSparkMaxSim.kPeriod);
      scoringMotor.update(
          m_scoringElevatorSim.getVelocityMetersPerSecond()
              * TrapElvConstants.ELV_GEAR_RATIO
              / Units.inchesToMeters(1));

      m_wristMotorSim.setInput(wristMotor.get() * RobotController.getBatteryVoltage());
      m_wristMotorSim.update(CANSparkMaxSim.kPeriod);
      wristMotor.update(m_wristMotorSim.getVelocityRadPerSec());
    }

    SmartDashboard.putNumber("base CAN Sim", baseMotor1.get());
    SmartDashboard.putNumber("scoring CAN Sim", scoringMotor.get());
    SmartDashboard.putNumber("wristMotorSim", wristMotor.get());

    baseMech.setLength(m_baseElevatorSim.getPositionMeters());
    scoringMech.setLength(m_scoringElevatorSim.getPositionMeters());
    // wristMech.setAngle(Units.radiansToDegrees(m_wristMotorSim.getAngleRads()));

    SmartDashboard.putNumber(
        "Base Elv Length", Units.metersToInches(m_baseElevatorSim.getPositionMeters()));
    SmartDashboard.putNumber(
        "Scoring Elv Length", Units.metersToInches(m_scoringElevatorSim.getPositionMeters()));
    SmartDashboard.putNumber("Wrist Angle", Units.radiansToDegrees(m_wristMotorSim.getAngleRads()));
  }
}
