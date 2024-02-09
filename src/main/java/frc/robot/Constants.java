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
    public static final int MOTOR_ID = 9; // edit all constants when testing
    public static final double LOAD_PERCENTAGE = 0.5; // used when intaking into the turret
    public static final double HOLD_PERCENTAGE =
        0.05; // very slow motor speed in case note slips out of trigger
    public static final double SHOOT_PERCENTAGE =
        -0.5; // used when feeding note into turret to fire (should be negative value because it
    // outtakes)
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_ID = 55;
    public static final int SHOOTER_MOTOR_RIGHT_ID = 56;

    // Placeholder values
    public static final double SHOOTER_LEFT_P = 0.0;
    public static final double SHOOTER_LEFT_I = 0.0;
    public static final double SHOOTER_LEFT_D = 0.0;
    public static final double SHOOTER_LEFT_FF = 0.0;

    public static final double SHOOTER_RIGHT_P = 0.0;
    public static final double SHOOTER_RIGHT_I = 0.0;
    public static final double SHOOTER_RIGHT_D = 0.0;
    public static final double SHOOTER_RIGHT_FF = 0.0;

    // Motor RPM, NOT roller RPM
    public static final double SHOOTER_IDLE_SPEED_LEFT = 100; // Placeholder; in RPM
    public static final double SHOOTER_IDLE_SPEED_RIGHT = 100; // Placeholder; in RPM

    public static final double SHOOTER_SPEED_TOLERANCE =
        0.1; // Placeholder; speed must be within (1-n)v to (1+n)v to fire

    public static final double SHOOTER_LEFT_GEARING = 2.5; // Unitless
    public static final double SHOOTER_LEFT_MOMENT = 0.000848475500006; // Placeholder; in kg*m^2

    public static final double SHOOTER_RIGHT_GEARING = 2.5; // Unitless
    public static final double SHOOTER_RIGHT_MOMENT = 0.000848475500006; // Placeholder; in kg*m^2
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
