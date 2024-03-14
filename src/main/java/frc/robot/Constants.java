// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem.SpeakerConfig;
import frc.robot.utilities.HowdyFF;
import frc.robot.utilities.HowdyPID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveDriveConstants {
    public static final double TURN_kP = 10;
    public static final double TURN_kD = 0;
    public static final double MAX_AUTO_TURN = 180; // deg/s
    public static final double MAX_AUTO_ACCERLATION = 180; // deg/s^2
    public static final double LOW_GEAR_MAG_MULTIPLE = 0.6;
    public static final double LOW_GEAR_STEER_MULTIPLE = 1;
    public static final double HIGH_GEAR_MAG_MULTIPLE = 1;
    public static final double HIGH_GEAR_STEER_MULTIPLE = 1;
    public static final double TRANSLATION_DEADBAND = 0.1;
    public static final double ROTATION_DEADBAND = 0.01;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 10;
    public static final int INTAKE_CHOOSER_ID = 14;
    public static final double INTAKE_PERCENTAGE = 0.25;
    public static final double CHOOSER_PERCENTAGE = 1;
    public static final double AMP_INTAKE_PERCENTAGE = 0.5;
  }

  public static class TriggerConstants {
    public static final int MOTOR_ID = 13; // edit all constants when testing
    public static final double LOAD_PERCENTAGE = -1; // used when intaking into the turret
    public static final double HOLD_PERCENTAGE =
        -0.05; // very slow motor speed in case note slips out of trigger
    public static final double SHOOT_PERCENTAGE =
        1; // used when feeding note into turret to fire (should be negative value because it
    // outtakes)
    public static final boolean MOTOR_INVERT = false;
    public static final double EJECT_PERCENT = -0.25;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_ID = 1;
    public static final int SHOOTER_MOTOR_RIGHT_ID = 4;

    // PID
    public static final HowdyPID RIGHT_SHOOTER_PID = new HowdyPID(0.0003, 0, 0, 0, 0.0002);
    public static final HowdyPID LEFT_SHOOTER_PID = new HowdyPID(0.0003, 0, 0, 0, 0.0002);

    // Motor RPM, NOT roller RPM
    public static final double SHOOTER_IDLE_SPEED_LEFT = 400; // Placeholder; in RPM
    public static final double SHOOTER_IDLE_SPEED_RIGHT = 400; // Placeholder; in RPM

    public static final double SHOOTER_SPEED_TOLERANCE =
        0.05; // speed must be within (1-n)v to (1+n)v to fire

    public static final double SHOOTER_LEFT_GEARING = 0.4; // Unitless
    public static final double SHOOTER_LEFT_MOMENT = 0.000848475500006; // Placeholder; in kg*m^2

    public static final double SHOOTER_RIGHT_GEARING = 0.4; // Unitless
    public static final double SHOOTER_RIGHT_MOMENT = 0.000848475500006; // Placeholder; in kg*m^2
    public static final SpeakerConfig SHOOTER_SOURCE_INTAKE = new SpeakerConfig(-1, -1500, -1500);
    public static final double INTAKE_DELAY_SEC = 0;
    public static final int BEAM_BREAK_ID = 1;
    public static final double BEAM_BREAK_THRESHOLD = 150;
  }

  public static class TurretConstants {
    public static final int TURRET_MOTOR_ID = 2;
    public static final int TURRET_CANcoder_ID = 17; // replace with actual CANcoder ID
    public static final int PITCH_MOTOR_ID = 3;
    public static final int PITCH_ENCODER_ID = 1; // replace with actual ID
    public static final boolean IS_MOTOR_INVERTED = true;
    public static final int DEFAULT_SHOT_PITCH = 40;
    public static final int DEFAULT_SHOT_ROTATION = 0;

    // PID coefficients
    public static final boolean ADVANCE_LOOP = false;
    public static final HowdyPID TURRET_POSITION_PID_CASCADE = new HowdyPID(5, 0.01, 0.225, 0.05, 0);
    public static final HowdyPID TURRET_VELOCITY_PID_CASCADE = new HowdyPID(5, 0.01, 0.225, 0.05, 0);
    public static final HowdyPID TURRET_POSITION_PID = new HowdyPID(5, 0.01, 0.225, 0.05, 0);


    public static final double TURRET_KMAXOUTPUT = 1;
    public static final double TURRET_KMINOUTPUT = -1;

    public static final int TURRET_MIN_ANGLE_DEGREES = -30;
    public static final int TURRET_MAX_ANGLE_DEGREES = 30; // 56.2 - 11 measured

    public static final double TURRET_CONVERSION_FACTOR = 0.18292682926829268292682926829268;
    public static final int TURRET_SMART_CURRENT_LIMIT = 40;

    // TODO: Change values when there's an actual real functional robot.
    public static final HowdyPID PITCH_PID = new HowdyPID(8, 0, 0);
    public static final HowdyFF PITCH_FF = new HowdyFF(0, 0.3, 0.1);

    public static final double PITCH_KMAXOUTPUT = 1;
    public static final double PITCH_KMINOUTPUT = -1;

    public static final int PITCH_MAX_ANGLE_DEGREES = 70;
    public static final int PITCH_MIN_ANGLE_DEGREES = -5;

    public static final double PITCH_CONVERSION_FACTOR = 128;
    public static final int PITCH_SMART_CURRENT_LIMIT = 40;

    public static final double PITCH_ZERO_OFFSET = Units.degreesToRotations(265.72);

    // Physics Values
    public static final double SHOOTER_CENTER_OF_GRAVITY = 1; // TODO: Get real values
    public static final double SHOOTER_MASS = 1;

    // Hardcoded Setpoints
    public static final double TURRET_STOWED_ANGLE = 0;
    public static final double PITCH_STOWED_ANGLE = 30;

    public static final double TURRET_PICKUP_ANGLE = 0;
    public static final double PITCH_PICKUP_ANGLE = 30;

    public static final double SPEAKER_TAG_CENTER_HEIGHT_METERS =
        1.450975; // Don't change unless FIRST changes the field layout
    public static final int SPEAKER_TAG_ID_RED = 4;
    public static final int SPEAKER_TAG_ID_BLUE = 7;

    // PID coefficients
    public static final double KMAXOUTPUT = 1;
    public static final double KMINOUTPUT = -1;
    public static final int MAX_TURRET_ANGLE_DEGREES = 110;
    public static final double TURRET_MOTOR_TURRET_RATIO = 0.2;

    public static final int TURRET_GEAR_TEETH = 164;
    public static final int LOW_GEAR_CANCODER_TEETH = 13;
    public static final int HIGH_GEAR_CANCODER_TEETH = 10;

    public static final double HIGH_GEAR_CAN_CODER_RATIO =
        TURRET_GEAR_TEETH
            / (HIGH_GEAR_CANCODER_TEETH
                + 0.0); // Revolutions of the turret, to revolutions of the cancoder
    public static final double LOW_GEAR_CAN_CODER_RATIO =
        TURRET_GEAR_TEETH
            / (LOW_GEAR_CANCODER_TEETH
                + 0.0); // Revolutions of the turret, to revolutions of the cancoder
    public static final int highGearCAN_CODER_ID = 5;
    public static final int lowGearCAN_CODER_ID = 6;
    public static final double ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV = 0.25;

    // Turret limits
    public static final double TURRET_MIN_ANGLE_ROTATIONS =
        (TURRET_MIN_ANGLE_DEGREES / (360 * TURRET_MOTOR_TURRET_RATIO));
    public static final double TURRET_MAX_ANGLE_ROTATIONS =
        (TURRET_MAX_ANGLE_DEGREES / (360 * TURRET_CONVERSION_FACTOR));
    public static final double PITCH_MIN_ANGLE_ROTATIONS =
        ((PITCH_MIN_ANGLE_DEGREES * PITCH_CONVERSION_FACTOR) / 360);
    public static final double PITCH_MAX_ANGLE_ROTATIONS =
        ((PITCH_MAX_ANGLE_DEGREES * PITCH_CONVERSION_FACTOR) / 360);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double RUMBLE_STRENGTH = .5;
  }

  public static class TrapElvConstants {
    // Control
    public static final double INTAKE_BEAM_BREAK_DELAY_SEC = 0.025;
    public static final double SOURCE_BEAM_BREAK_DELAY_SEC = 0.15;

    // Wrist
    public static final int WRIST_MOTOR_ID = 12;
    public static final int WRIST_ENCODER_ID = 25;
    public static final int SOURCE_BREAK_ID = 2;
    public static final int GROUND_BREAK_ID = 3;

    public static final HowdyPID WRIST_PID = new HowdyPID(0.03, 0, 0);
    public static final HowdyFF WRIST_FF = new HowdyFF(0, 0.54, 4.28, 0.05);

    public static final double WRIST_MIN_ANGLE = Units.degreesToRadians(-90); // RADS
    public static final double WRIST_MAX_ANGLE = Units.degreesToRadians(270); // RADS
    public static final double WRIST_LENGTH = Units.inchesToMeters(11.877934);
    public static final double WRIST_MOI = 0.3175242664; // Moment of Inertia
    public static final double WRIST_GEAR_RATIO = 35;
    public static final double WRIST_ZERO_OFFSET = 0.8164931;
    public static final double WRIST_DEADZONE = 0.01;

    // Roller
    public static final int ROLLER_MOTOR_ID = 15;

    public static final double ROLLER_SPEED = 0.4;
    public static final double ROLLER_REVERSE_SPEED = -0.5;

    public static final double ROLLER_DEADZONE = 0.02;

    // Elv
    public static final int BASE_MOTOR1_ID = 22;
    public static final int BASE_MOTOR2_ID = 23;
    public static final int SCORING_MOTOR_ID = 24;
    public static final int BASE_BREAK_ID = 3;
    public static final int SCORING_BREAK_ID = 4;

    public static final double ELV_ZEROING_SPEED = 0.1; // Percent Power
    public static final HowdyPID BASE_PID = new HowdyPID(36e-3, 5e-7, 1e-4, 0.0, 2e-6);
    public static final HowdyPID SCORING_PID = new HowdyPID(36e-3, 5e-7, 1e-4, 0.0, 2e-6);

    public static final int ELV_GEAR_RATIO = 70;
    public static final double ELV_LIFT_MASS = 5.4; // kg
    public static final double ELV_MIN_HEIGHT = Units.inchesToMeters(12);
    public static final double ELV_MAX_HEIGHT = Units.inchesToMeters(30);
    public static final double DRUM_RADIUS = Units.inchesToMeters(1);

    public static final double BREAK_THRESHOLD_MM = 200;
  }

  public static class LimelightConstants {
    public static final double FIELD_LENGTH = (8.308467 * 2);
    public static final double FIELD_HALF_WIDTH = 3.837865;
    public static final double TAG_Y_POS = 1.451102;
    public static final double TAG_HEIGHT = 0.4572;

    public static final int SPEAKER_TAG_ID_RED = 4;
    public static final int SPEAKER_TAG_ID_BLUE = 7;
  }

  public static class VisionConstants {
    public static final double TURRET_LIMELIGHT_HEIGHT_INCHES = 17.85;
    public static final double MAX_ACCEPTABLE_ERROR_METERS = 2;
    public static final double MAX_TIME_BETWEEN_POSES_SECONDS = 0.2; // 10 periodic cycles

    public static final String MAIN_CAMERA_NAME = "Camera_Module_v1";
    public static final String TURRET_CAMERA_NAME = "Camera_Module_Turret";
  }

  public static class ClimberConstants {
    public static final int LEFT_ARM_ID = 20;
    public static final int RIGHT_ARM_ID = 11;
    public static final double GEAR_RATIO = 175;
    public static final double CLIMB_PERCENT = -0.4;
    public static final double CLIMB_POSITION = 7;
    public static final double[] POSITION_PID = new double[] {0.15, 0, 0, 0};
    public static final double[] CURRENT_PID = new double[] {0.01, 0, 0, 0};
    public static final double MIN_RAISE_TIME_SEC = 0.2;
    public static final double CLIP_CURRENT = -10;
    public static final double RAISE_CURRENT = 30;
    public static final double INITAL_RAISE_PERCENT = 0.3;
    public static final double BREAK_STATIC_PERCENT = -0.3;
    public static final double BREAK_STATIC_TIME = 0.1;
  }

  public static class FieldConstants {
    public static final Translation2d RED_SPEAKER = new Translation2d(16.579342, 5.547868);
    public static final Translation2d BLUE_SPEAKER = new Translation2d(-0.0381, 5.547868);
    public static final double CENTERLINE_X_APPROX = 8;
  }

  public static class enabledSubsystems {
    public static final boolean intakeEnabled = true;
    public static final boolean drivetrainEnabled = true;
    public static final boolean visionEnabled = false;
    public static final boolean usingPhoton = false;
    public static final boolean elvEnabled = true;
    public static final boolean signalEnabled = false;
    public static final boolean shooterEnabled = true;
    public static final boolean triggerEnabled = true;
    public static final boolean turretRotationEnabled = true;
    public static final boolean turretPitchEnabled = true;
    public static final boolean climberEnabled = false;
  }

  public static final int END_GAME_WARNING_TIME = 20;
  public static final double TELEMETRY_LOG_NUMBER = .2;
  // Rumble
  public static final double AMPLIFICATION_RUMBLE_TIME = 0.5;
  public static final double AMPLIFICATION_RUMBLE_INTENSITY = 0.5;

  // Lights
  public static final int LED_COUNT = 20;

  public static class DriverConstants {

    public static final Rotation2d RED_AMP_ROTATION = Rotation2d.fromRotations(0.25);
    public static final Rotation2d BLUE_AMP_ROTATION = Rotation2d.fromRotations(0.25);
    public static final Rotation2d BLUE_SOURCE_ROTATION = Rotation2d.fromRotations(-0.25);
    public static final Rotation2d RED_SOURCE_ROTATION = Rotation2d.fromRotations(-0.25);
  }

  public static class CommandConstants {

    public static final double WAIT_FOR_TRAPELV = 0.1;
  }
}
