// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double CONVERSION_FACTOR = 0.25; // for the revbot prototype turret
    public static final int MAXRPM = 5700;
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
