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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapArmConstants;
import frc.robot.Robot;
import frc.robot.networktables.DebugEntry;

public class TrapArmSubsystem extends SubsystemBase {
  // Wrist motors
  private final CANSparkMax wristMotor;
  private final CANSparkMax rollerMotor;

  // Elevator motors
  private final CANSparkMaxSim baseMotor1;
  private final CANSparkMax baseMotor2;
  private final CANSparkMaxSim scoringMotor;

  // Beam Breaks
  private final DigitalInput sourceBreak;
  private final DigitalInput groundBreak;

  // Limit Switches
  private final DigitalInput baseBreak;
  private final DigitalInput scoringBreak;

  // Encoders
  private final CANcoder wristEncoder;

  // PID
  // P, I, D, Iz, FF
  private double[] basePID = {36e-3, 5e-7, 1e-4, 0.0, 2e-6};
  private double[] scoringPID = {36e-3, 5e-7, 1e-4, 0.0, 2e-6};

  private DebugEntry<Boolean> sourceLog;
  private DebugEntry<Boolean> groundLog;
  private DebugEntry<Boolean> baseLog;
  private DebugEntry<Boolean> scoringLog;

  private Mechanism2d armMechanism;
  private MechanismRoot2d root;
  private MechanismLigament2d baseMech;
  private MechanismLigament2d scoringMech;
  private MechanismLigament2d wristMech;

  private ElevatorSim m_baseElevatorSim;
  private ElevatorSim m_scoringElevatorSim;

  private ShuffleboardTab TrapArmTab = Shuffleboard.getTab("Trap Arm Tab");
  private GenericEntry basePIDEntry = TrapArmTab.add("Base PID", basePID).getEntry();
  private GenericEntry scoringPIDEntry = TrapArmTab.add("Scoring PID", scoringPID).getEntry();
  private GenericEntry baseGoal = TrapArmTab.add("Base Goal", 0).getEntry();

  // States
  public static enum TrapArmState {
    STOWED(0.0, 0.0, 0.0),
    FROM_INTAKE(15.0, 0.0, 0.0),
    FROM_SOURCE(150.0, 0.0, 12.0),
    TRAP_SCORE(0.0, 12.0, 12.0),
    AMP_SCORE(90.0, 0.0, 12.0);

    private Double wristPose;
    private Double basePose;
    private Double scoringPose;

    TrapArmState(Double wrist, Double base, Double scoring) {
      this.wristPose = wrist;
      this.basePose = base;
      this.scoringPose = scoring;
    }

    private double feetToRotations(double f) {
      return Units.inchesToMeters(f) / (2 * Math.PI * Units.inchesToMeters(1)) * 70;
    }

    private double angleToRotations(double a) {
      return a / 360;
    }

    public Double getWristPose() {
      return angleToRotations(wristPose);
    }

    public Double getBasePose() {
      return feetToRotations(basePose);
    }

    public Double getScoringPose() {
      return feetToRotations(scoringPose);
    }
  }

  /** Creates a new TrapArm. */
  public TrapArmSubsystem() {
    // Wrist
    wristMotor = new CANSparkMax(TrapArmConstants.wristMotor_ID, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    wristEncoder = new CANcoder(6);

    rollerMotor = new CANSparkMax(TrapArmConstants.rollerMoter_ID, MotorType.kBrushless);
    rollerMotor.restoreFactoryDefaults();

    sourceBreak = new DigitalInput(TrapArmConstants.sourceBreak_ID);
    groundBreak = new DigitalInput(TrapArmConstants.groundBreak_ID);

    // Arm

    baseMotor1 = new CANSparkMaxSim(TrapArmConstants.baseMotor1_ID, MotorType.kBrushless);
    baseMotor1.restoreFactoryDefaults();
    baseMotor1.getPIDController().setP(basePID[0]);
    baseMotor1.getPIDController().setI(basePID[1]);
    baseMotor1.getPIDController().setD(basePID[2]);
    baseMotor1.getPIDController().setIZone(basePID[3]);
    baseMotor1.getPIDController().setFF(basePID[4]);
    TrapArmTab.add("Base Elv PID", baseMotor1.getPIDController());

    baseMotor2 = new CANSparkMax(TrapArmConstants.baseMotor2_ID, MotorType.kBrushless);
    baseMotor2.restoreFactoryDefaults();
    baseMotor2.getPIDController().setP(basePID[0]);
    baseMotor2.getPIDController().setI(basePID[1]);
    baseMotor2.getPIDController().setD(basePID[2]);
    baseMotor2.getPIDController().setIZone(basePID[3]);
    baseMotor2.getPIDController().setFF(basePID[4]);

    baseBreak = new DigitalInput(TrapArmConstants.baseBreak_ID);

    scoringMotor = new CANSparkMaxSim(TrapArmConstants.scoringMotor_ID, MotorType.kBrushless);
    scoringMotor.restoreFactoryDefaults();
    scoringMotor.getPIDController().setP(scoringPID[0]);
    scoringMotor.getPIDController().setI(scoringPID[1]);
    scoringMotor.getPIDController().setD(scoringPID[2]);
    scoringMotor.getPIDController().setIZone(scoringPID[3]);
    scoringMotor.getPIDController().setFF(scoringPID[4]);
    TrapArmTab.add("Scoring Elv PID", scoringMotor.getPIDController());

    scoringBreak = new DigitalInput(TrapArmConstants.scoringBreak_ID);

    // SmartDashboard
    sourceLog = new DebugEntry<Boolean>(baseBreak.get(), "Source Beam Break", this);
    groundLog = new DebugEntry<Boolean>(baseBreak.get(), "Ground Beam Break", this);
    baseLog = new DebugEntry<Boolean>(baseBreak.get(), "Base Limit Switch", this);
    scoringLog = new DebugEntry<Boolean>(baseBreak.get(), "Scoring Limit Switch", this);

    basePIDEntry.setDoubleArray(basePID);
    scoringPIDEntry.setDoubleArray(scoringPID);

    // Simulation
    if (Robot.isSimulation()) {
      // 2D Mechanism
      armMechanism = new Mechanism2d(2, 2);
      root = armMechanism.getRoot("Root", 1, 0);
      baseMech =
          root.append(new MechanismLigament2d("Base Arm", 0, 90, 20, new Color8Bit(Color.kBlue)));
      scoringMech =
          baseMech.append(
              new MechanismLigament2d("Scoring Arm", 0, 0, 10, new Color8Bit(Color.kAqua)));
      wristMech =
          scoringMech.append(
              new MechanismLigament2d("Wrist Mech", 1.5, -170, 5, new Color8Bit(Color.kRed)));

      m_baseElevatorSim =
          new ElevatorSim(
              DCMotor.getNEO(1),
              TrapArmConstants.elevatorGearRatio,
              TrapArmConstants.elevatorCariageMass,
              Units.inchesToMeters(1),
              TrapArmConstants.elevatorMinHight,
              TrapArmConstants.elevatorMaxHight,
              true,
              0);

      m_scoringElevatorSim =
          new ElevatorSim(
              DCMotor.getNEO(1),
              TrapArmConstants.elevatorGearRatio,
              TrapArmConstants.elevatorCariageMass,
              Units.inchesToMeters(1),
              TrapArmConstants.elevatorMinHight,
              TrapArmConstants.elevatorMaxHight,
              true,
              0);

      TrapArmTab.add("Trap Arm Mech", armMechanism);
    }
  }

  // Commands
  public Command intakeSource() {
    return startEnd(
        () -> {
          if (!sourceBreak.get()) {
            setTrapArm(TrapArmState.FROM_SOURCE);
            rollerMotor.set(TrapArmConstants.rollerIntakeSpeed);
          }
        },
        () -> {
          rollerMotor.stopMotor();
        });
  }

  public Command intakeGround() {
    return startEnd(
        () -> {
          if (!groundBreak.get()) {
            setTrapArm(TrapArmState.FROM_INTAKE);
            rollerMotor.set(-TrapArmConstants.rollerIntakeSpeed);
          }
        },
        () -> {
          rollerMotor.stopMotor();
        });
  }

  public Command scoreAMP() {
    return startEnd(
        () -> {
          setTrapArm(TrapArmState.AMP_SCORE);
          rollerMotor.set(-TrapArmConstants.rollerScoringSpeed);
          if (Robot.isSimulation()) {
            wristMech.setAngle(-70);
          }
        },
        () -> {
          rollerMotor.stopMotor();
          if (Robot.isSimulation()) {
            wristMech.setAngle(-170);
          }
        });
  }

  public Command scoreTrap() {
    return startEnd(
        () -> {
          setTrapArm(TrapArmState.TRAP_SCORE);
          rollerMotor.set(TrapArmConstants.rollerScoringSpeed);
          if (Robot.isSimulation()) {
            // baseMech.setLength(baseMech.getLength() + 1);
            // scoringMech.setLength(scoringMech.getLength() + 1);
            // wristMech.setAngle(-90);
          }
        },
        () -> {
          rollerMotor.stopMotor();
          if (Robot.isSimulation()) {
            // baseMech.setLength(baseMech.getLength() - 1);
            // scoringMech.setLength(scoringMech.getLength() - 1);
            // wristMech.setAngle(-170);
          }
        });
  }

  public Command zeroArm() {
    return startEnd(
        () -> {
          // Command for zering elevator if elevator happens to be not at zero
          // Runs elevator motors until there limit switches are pressed
          if (!baseBreak.get()) {
            baseMotor1.set(.1);
            baseMotor2.set(.1);
          } else {
            baseMotor1.stopMotor();
            baseMotor2.stopMotor();
          }
          if (!scoringBreak.get()) {
            scoringMotor.set(.1);
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

  public Command setTrapArm(TrapArmState state) {
    return startEnd(
        () -> {
          baseGoal.setDouble(
              Units.metersToInches(TrapArmConstants.elevatorMinHight) + state.basePose);
          wristMotor.getPIDController().setReference(state.getWristPose(), ControlType.kPosition);
          baseMotor1.getPIDController().setReference(state.getBasePose(), ControlType.kPosition);
          baseMotor2.getPIDController().setReference(state.getBasePose(), ControlType.kPosition);
          scoringMotor
              .getPIDController()
              .setReference(state.getScoringPose(), ControlType.kPosition);
        },
        () -> {
          wristMotor.stopMotor();
          baseMotor1.getPIDController().setReference(0, ControlType.kPosition);
          baseMotor2.getPIDController().setReference(0, ControlType.kPosition);
          scoringMotor.getPIDController().setReference(0, ControlType.kPosition);
        });
  }

  @Override
  public void periodic() {
    sourceLog.log(sourceBreak.get());
    groundLog.log(groundBreak.get());
    baseLog.log(baseBreak.get());
    scoringLog.log(sourceBreak.get());
  }

  @Override
  public void simulationPeriodic() {
    for (double i = 0; i < Robot.defaultPeriodSecs; i += CANSparkMaxSim.kPeriod) {
      m_baseElevatorSim.setInput(baseMotor1.get() * RobotController.getBatteryVoltage());
      m_baseElevatorSim.update(CANSparkMaxSim.kPeriod);
      baseMotor1.update(
          m_baseElevatorSim.getVelocityMetersPerSecond() * 70 / Units.inchesToMeters(1));

      m_scoringElevatorSim.setInput(scoringMotor.get() * RobotController.getBatteryVoltage());
      m_scoringElevatorSim.update(CANSparkMaxSim.kPeriod);
      scoringMotor.update(
          m_scoringElevatorSim.getVelocityMetersPerSecond()
              * TrapArmConstants.elevatorGearRatio
              / Units.inchesToMeters(1));
    }
    SmartDashboard.putNumber("base CAN Sim", baseMotor1.get());
    SmartDashboard.putNumber("scoring CAN Sim", scoringMotor.get());
    baseMech.setLength(m_baseElevatorSim.getPositionMeters());
    scoringMech.setLength(m_scoringElevatorSim.getPositionMeters());
    SmartDashboard.putNumber(
        "Base Elv Length", Units.metersToInches(m_baseElevatorSim.getPositionMeters()));
    SmartDashboard.putNumber(
        "Scoring Elv Length", Units.metersToInches(m_scoringElevatorSim.getPositionMeters()));
  }
}
