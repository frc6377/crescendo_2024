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

  public static class TurretConstants {
    public static final int MOTOR_ID = 9;
    public static final int CANcoder_ID = 17; // replace with actual CANcoder ID

    // PID coefficients
    public static final double KP = 5e-5;
    public static final double KI = 1e-6;
    public static final double KD = 0;
    public static final double KIZ = 0;
    public static final double KFF = 0.000156;
    public static final double KMAXOUTPUT = 1;
    public static final double KMINOUTPUT = -1;
    public static final int MAX_TURRET_ANGLE_DEGREES = 110;
    public static final double CONVERSION_FACTOR = 0.25; // for the revbot prototype turret
    public static final int MAXRPM = 5700;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TrapElvConstants {
    // IDs | TODO: get real device IDs
    public static final int WRIST_MOTOR_ID = 1;
    public static final int ROLLER_MOTOR_ID = 2;
    public static final int BASE_MOTOR1_ID = 3;
    public static final int BASE_MOTOR2_ID = 4;
    public static final int SCORING_MOTOR_ID = 5;
    public static final int WRIST_ENCODER_ID = 6;
    public static final int SOURCE_BREAK_ID = 1;
    public static final int GROUND_BREAK_ID = 2;
    public static final int BASE_BREAK_ID = 3;
    public static final int SCORING_BREAK_ID = 4;

    // Speeds
    public static final double ROLLER_INTAKE_SPEED = 0.25;
    public static final double ROLLER_SCORING_SPEED = 0.4;
    public static final double ELV_ZEROING_SPEED = 0.1; // Percent Power

    // PIDs
    // P, I, D, Iz, FF
    public static final double[] BASE_PID = {36e-3, 5e-7, 1e-4, 0.0, 2e-6};
    public static final double[] SCORING_PID = {36e-3, 5e-7, 1e-4, 0.0, 2e-6};
    public static final double[] WRIST_PID = {36e-2, 5e-5, 1e-4, 0.0, 0};

    // Simulation
    public static final int ELV_GEAR_RATIO = 70;
    public static final int WRIST_GEAR_RATIO = 70;
    public static final double ELV_LIFT_MASS = 5.4; // kg
    public static final double DRUM_RADIUS = Units.inchesToMeters(1);
    public static final double ELV_MIN_HEIGHT = Units.inchesToMeters(12);
    public static final double ELV_MAX_HEIGHT = Units.inchesToMeters(30);

    public static final double WRIST_MIN_ANGLE = Units.degreesToRadians(-359); // RADS
    public static final double WRIST_MAX_ANGLE = Units.degreesToRadians(359); // RADS
    public static final double WRIST_LENGTH = Units.inchesToMeters(12.5);
  }

  public static final int END_GAME_WARNING_TIME = 20;

  // Rumble
  public static final double AMPLIFICATION_RUMBLE_TIME = 0.5;
  public static final double AMPLIFICATION_RUMBLE_INTENSITY = 0.5;

  // Lights
  public static final int LED_COUNT = 20;
}
