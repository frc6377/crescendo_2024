// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapArmConstants;

public class TrapArmSubsystem extends SubsystemBase {
  // Wrist motors
  private final CANSparkMax wristMotor;
  private final CANSparkMax rollerMotor;

  // Elevator motors
  private final CANSparkMax baseMotor1;
  private final CANSparkMax baseMotor2;
  private final CANSparkMax scoringMotor;

  // Beam Breaks
  private final DigitalInput sourceBreak;
  private final DigitalInput groundBreak;

  // Limit Switches
  private final DigitalInput baseBreak;
  private final DigitalInput scoringBreak;

  // Encoders
  private final CANcoder wristEncoder;

  // PID
  //                             P      I     D     Iz   FF
  private Double[] basePID = {36e-5, 5e-7, 1e-4, 0.0, 2e-6};
  private Double[] scoringPID = {36e-5, 5e-7, 1e-4, 0.0, 2e-6};

  // States
  public static enum TrapArmState {
    STOWED(0.0, 0.0, 0.0),
    FROM_INTAKE(0.3, 0.0, 0.0),
    FROM_SOURCE(0.4, 0.0, 1.0),
    TRAP_SCORE(0.0, 1.0, 2.0),
    AMP_SCORE(0.0, 0.0, 0.0);

    private Double wristPose;
    private Double basePose;
    private Double scoringPose;

    TrapArmState(Double wrist, Double base, Double scoring) {
      this.wristPose = wrist;
      this.basePose = base;
      this.scoringPose = scoring;
    }

    public Double getWristPose() {
      return wristPose;
    }

    public Double getBasePose() {
      return basePose;
    }

    public Double getScoringPose() {
      return scoringPose;
    }
  };

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

    // Elevator
    baseMotor1 = new CANSparkMax(TrapArmConstants.baseMotor1_ID, MotorType.kBrushless);
    baseMotor1.restoreFactoryDefaults();
    baseMotor1.getPIDController().setP(basePID[0]);
    baseMotor1.getPIDController().setI(basePID[1]);
    baseMotor1.getPIDController().setD(basePID[2]);
    baseMotor1.getPIDController().setIZone(basePID[3]);
    baseMotor1.getPIDController().setFF(basePID[4]);

    baseMotor2 = new CANSparkMax(TrapArmConstants.baseMotor2_ID, MotorType.kBrushless);
    baseMotor2.restoreFactoryDefaults();
    baseMotor2.getPIDController().setP(basePID[0]);
    baseMotor2.getPIDController().setI(basePID[1]);
    baseMotor2.getPIDController().setD(basePID[2]);
    baseMotor2.getPIDController().setIZone(basePID[3]);
    baseMotor2.getPIDController().setFF(basePID[4]);

    baseBreak = new DigitalInput(TrapArmConstants.baseBreak_ID);

    scoringMotor = new CANSparkMax(TrapArmConstants.scoringMotor_ID, MotorType.kBrushless);
    scoringMotor.restoreFactoryDefaults();
    scoringMotor.getPIDController().setP(scoringPID[0]);
    scoringMotor.getPIDController().setI(scoringPID[1]);
    scoringMotor.getPIDController().setD(scoringPID[2]);
    scoringMotor.getPIDController().setIZone(scoringPID[3]);
    scoringMotor.getPIDController().setFF(scoringPID[4]);

    scoringBreak = new DigitalInput(TrapArmConstants.scoringBreak_ID);

    // Smartdashboard
    SmartDashboard.putNumberArray("Base PID", basePID);
    SmartDashboard.putNumberArray("Scoring PID", scoringPID);
  }

  // Commands
  public Command intakeSource() {
    return startEnd(
        () -> {
            setTrapArm(TrapArmState.FROM_SOURCE);
            rollerMotor.set(TrapArmConstants.rollerIntakeSpeed);
        },

        () -> { stop(); }
        );
  }

  public Command intakeGround() {
    return startEnd(
        () -> {
            setTrapArm(TrapArmState.FROM_INTAKE);
            rollerMotor.set(-TrapArmConstants.rollerIntakeSpeed);
        },

        () -> { stop(); }
        );
  }

  public Command scoreAMP() {
    return startEnd(
        () -> {
            setTrapArm(TrapArmState.AMP_SCORE);
            rollerMotor.set(TrapArmConstants.rollerScoringSpeed);
        },

        () -> { stop(); }
        );
  }

  public Command scoreTrap() {
    return startEnd(
        () -> {
          setTrapArm(TrapArmState.TRAP_SCORE);
          rollerMotor.set(TrapArmConstants.rollerScoringSpeed);
        },
        
        () -> { stop(); }
        );
  }

  public Command setTrapArm(TrapArmState state) {
    return run(
        () -> {
          wristMotor.getPIDController().setReference(state.getWristPose(), ControlType.kPosition);
          baseMotor1.getPIDController().setReference(state.getBasePose(), ControlType.kPosition);
          baseMotor2.getPIDController().setReference(state.getBasePose(), ControlType.kPosition);
          scoringMotor
              .getPIDController()
              .setReference(state.getScoringPose(), ControlType.kPosition);
        });
  }

  public Command stop() {
    return run(
        () -> {
          wristMotor.stopMotor();
          baseMotor1.stopMotor();
          baseMotor2.stopMotor();
          scoringMotor.stopMotor();
          rollerMotor.stopMotor();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
