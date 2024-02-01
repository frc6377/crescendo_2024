// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;

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

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_TOP_ID = 55;
    public static final int SHOOTER_MOTOR_BOTTOM_ID = 56;

    // Placeholder values
    public static final double SHOOTER_P = 0.0;
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_FF = 0.0;

    // Top is index 0, bottom is index 1
    public static final Pair<Double, Double> SHOOTER_IDLE_SPEEDS = new Pair(150, 100); // Placeholder; in RPM
    public static final double SHOOTER_SPEED_TOLERANCE =
        0.1; // Placeholder; speed must be within (1-n)v to (1+n)v to fire
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final int END_GAME_WARNING_TIME = 20;

  // Rumble
  public static final double AMPLIFICATION_RUMBLE_TIME = 0.5;
  public static final double AMPLIFICATION_RUMBLE_INTENSITY = 0.5;

  // Lights
  public static final int LED_COUNT = 20;
}
