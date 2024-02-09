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
    // TODO: Get real CAN IDs
    public static final int INTAKE_MOTOR_ID = 6;
    public static final int INTAKE_CHOOSER_ID = 7;
    public static final double INTAKE_PERCENTAGE = -0.75;
    public static final double CHOOSER_PERCENTAGE = -0.75;
  }

  public static class TriggerConstants {
    public static final int MOTOR_ID = 8; // edit all constants when testing
    public static final double LOAD_PERCENTAGE = 0.5; // used when intaking into the turret
    public static final double HOLD_PERCENTAGE =
        0.05; // very slow motor speed in case note slips out of trigger
    public static final double SHOOT_PERCENTAGE =
        -0.5; // used when feeding note into turret to fire (should be negative value because it
    // outtakes)
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_TOP_ID = 55;
    public static final int SHOOTER_MOTOR_BOTTOM_ID = 56;

    // Placeholder values
    public static final double SHOOTER_P = 0.0;
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_FF = 0.0;

    // Top is index 0, bottom is index 1
    public static final double SHOOTER_IDLE_SPEED_TOP = 150; // Placeholder; in RPM
    public static final double SHOOTER_IDLE_SPEED_BOTTOM = 100; // Placeholder; in RPM
    public static final double SHOOTER_SPEED_TOLERANCE =
        0.1; // Placeholder; speed must be within (1-n)v to (1+n)v to fire
  }

  public static class TurretConstants {
    public static final int TURRET_MOTOR_ID = 9;
    public static final int TURRET_CANcoder_ID = 17; // replace with actual CANcoder ID
    public static final int pitch_MOTOR_ID = 10;
    public static final int pitch_CANcoder_ID = 18; // replace with actual CANcoder ID

    // PID coefficients
    public static final double TURRET_KP =
        0.25; // TODO: Change values when there's an actual real functional robot.
    public static final double TURRET_KI = 0.001;
    public static final double TURRET_KD = 0;
    public static final double TURRET_KIZ = 0;
    public static final double TURRET_KFF = 0;
    public static final double TURRET_KMAXOUTPUT = 1;
    public static final double TURRET_KMINOUTPUT = -1;
    public static final int TURRET_MAX_ANGLE_DEGREES = 110;
    public static final double TURRET_CONVERSION_FACTOR = 0.25;
    public static final int TURRET_SMART_CURRENT_LIMIT = 40;

    public static final double PITCH_KP =
        0.25; // TODO: Change values when there's an actual real functional robot.
    public static final double PITCH_KI = 0.001;
    public static final double PITCH_KD = 0;
    public static final double PITCH_KIZ = 0;
    public static final double PITCH_KFF = 0;
    public static final double PITCH_KMAXOUTPUT = 1;
    public static final double PITCH_KMINOUTPUT = -1;
    public static final int PITCH_MAX_ANGLE_DEGREES = 50;
    public static final int PITCH_MIN_ANGLE_DEGREES = 0;
    public static final double PITCH_CONVERSION_FACTOR = 0.25;
    public static final int PITCH_SMART_CURRENT_LIMIT = 40;

    public static final double SHOOTER_CENTER_OF_GRAVITY = 1;//TODO: Get real values
    public static final double SHOOTER_MASS = 1;
    public static final double PITCH_NEWTONS_TO_MOTOR_POWER = 1;

    // Limelight
    public static final double LIMELIGHT_HEIGHT_INCHES = 17.85;
    public static final double LIMELIGHT_PITCH_RADIANS = Math.toRadians(17.75);

    public static final double SPEAKER_TAG_CENTER_HEIGHT_INCHES =
        57.125; // Don't change unless FIRST changes the field layout
    public static final int SPEAKER_TAG_ID_RED = 4;
    public static final int SPEAKER_TAG_ID_BLUE = 7;
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
    public static final double[] WRIST_PID = {7, .1, .6, 0.0, 2e-6};

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
  public static final double TELEMETRY_LOG_NUMBER = .2;
  // Rumble
  public static final double AMPLIFICATION_RUMBLE_TIME = 0.5;
  public static final double AMPLIFICATION_RUMBLE_INTENSITY = 0.5;

  // Lights
  public static final int LED_COUNT = 20;
}
