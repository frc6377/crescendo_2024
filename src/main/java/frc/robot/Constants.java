// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 6;
    public static final double INTAKE_PERCENTAGE = -0.75;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TrapElvConstants {
    // IDs | TODO: get real device IDs
    public static final int wristMotor_ID = 1;
    public static final int rollerMoter_ID = 2;
    public static final int baseMotor1_ID = 3;
    public static final int baseMotor2_ID = 4;
    public static final int scoringMotor_ID = 5;
    public static final int wristEncoder_ID = 6;
    public static final int sourceBreak_ID = 1;
    public static final int groundBreak_ID = 2;
    public static final int baseBreak_ID = 3;
    public static final int scoringBreak_ID = 4;

    // Speeds
    public static final double rollerIntakeSpeed = 0.25;
    public static final double rollerScoringSpeed = 0.4;
    public static final int elvZeroingSpeed = 10; // RPM

    // Simulation
    public static final int elevatorGearRatio = 70;
    public static final double elevatorCariageMass = 5.4; // kg
    public static final double elevatorMinHight = Units.inchesToMeters(12);
    public static final double elevatorMaxHight = Units.inchesToMeters(30);
  }

  public static final int END_GAME_WARNING_TIME = 20;

  // Rumble
  public static final double AMPLIFICATION_RUMBLE_TIME = 0.5;
  public static final double AMPLIFICATION_RUMBLE_INTENSITY = 0.5;

  // Lights
  public static final int LED_COUNT = 20;
}
