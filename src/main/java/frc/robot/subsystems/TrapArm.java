// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapArm extends SubsystemBase {
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

  // States
  public static Boolean STOWED = false;
  public static Boolean FROM_INTAKE = false;
  public static Boolean FROM_SOURCE = false;
  public static Boolean TRAP_SCORE = false;
  public static Boolean AMP_SCORE = false;
  public static enum TrapArmState{
    STOWED,
    FROM_INTAKE,
    FROM_SOURCE,
    TRAP_SCORE,
    AMP_Score
  };

  public static HashMap<TrapArmState, Double> wristMotorPose;
  public static HashMap<TrapArmState, Double> rollerMotorSpeed;
  public static HashMap<TrapArmState, Double> baseMotorPose;
  public static HashMap<TrapArmState, Double> scoringMotorPose;



  /** Creates a new TrapArm. */
  public TrapArm() {
    // Wrist
    wristMotor = new CANSparkMax(1, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    wristEncoder = new CANcoder(6);

    rollerMotor = new CANSparkMax(2, MotorType.kBrushless);
    rollerMotor.restoreFactoryDefaults();

    sourceBreak = new DigitalInput(1);
    groundBreak = new DigitalInput(2);

    // Elevator
    baseMotor1 = new CANSparkMax(3, MotorType.kBrushless);
    baseMotor1.restoreFactoryDefaults();
    baseMotor2 = new CANSparkMax(4, MotorType.kBrushless);
    baseMotor2.restoreFactoryDefaults();
    baseMotor2.follow(baseMotor1);
    baseBreak = new DigitalInput(3);

    scoringMotor = new CANSparkMax(5, MotorType.kBrushless);     
    scoringBreak = new DigitalInput(4);

    // States
    wristMotorPose = new HashMap<TrapArmState, Double>() {{
      put(TrapArmState.STOWED, 0.25);
      put(TrapArmState.FROM_INTAKE, 0.3);
      put(TrapArmState.FROM_SOURCE, 0.4);
      put(TrapArmState.TRAP_SCORE, 0.0);
      put(TrapArmState.AMP_Score, 0.0);
    }};

    rollerMotorSpeed = new HashMap<TrapArmState, Double>() {{
      put(TrapArmState.STOWED, 0.0);
      put(TrapArmState.FROM_INTAKE, -0.25);
      put(TrapArmState.FROM_SOURCE, 0.25);
      put(TrapArmState.TRAP_SCORE, 0.25);
      put(TrapArmState.AMP_Score, -0.25);
    }};

    baseMotorPose = new HashMap<TrapArmState, Double>() {{
      put(TrapArmState.STOWED, 0.0);
      put(TrapArmState.FROM_INTAKE, 0.0);
      put(TrapArmState.FROM_SOURCE, 0.0);
      put(TrapArmState.TRAP_SCORE, 1.0);
      put(TrapArmState.AMP_Score, 0.25);
    }};

    scoringMotorPose = new HashMap<TrapArmState, Double>() {{
      put(TrapArmState.STOWED, 0.25);
      put(TrapArmState.FROM_INTAKE, 0.25);
      put(TrapArmState.FROM_SOURCE, 0.25);
      put(TrapArmState.TRAP_SCORE, 0.25);
      put(TrapArmState.AMP_Score, 0.25);
    }};

  }

  public Command fromSource() {
    return run(
      () -> {
        rollerMotor.set(-.25);
        wristEncoder.setPosition(.25);
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
