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
    public static final int TURRET_MOTOR_ID = 7;

    // PID coefficients
    public static final double TURRET_KP = 5e-5;
    public static final double TURRET_KI = 1e-6;
    public static final double TURRET_KD = 0;
    public static final double TURRET_KIZ = 0;
    public static final double TURRET_KFF = 0.000156;
    public static final double TURRET_KMAXOUTPUT = 1;
    public static final double TURRET_KMINOUTPUT = -1;
    public static final int TURRET_MAXRPM = 5700;

    // Smart Motion Coefficients
    public static final int TURRET_MAXVEL = 10; // rpm
    public static final int TURRET_MAXACC = 1;
    public static final int TURRET_SMARTMOTION_SLOT = 0;
    public static final int TURRET_MINVEL = 0; // replace 0 with the actual value
    public static final double TURRET_ALLOWEDERR = 0; // replace 0 with the actual value

  }


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
