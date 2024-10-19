package frc.robot.subsystems.swerveSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem.DriveRequest;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem.RotationSource;
import frc.robot.utilities.HowdyMath;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommandFactory {
  private final SwerveSubsystem subsystem;
  private final ArrayList<RotationSource> rotationSources;

  public SwerveCommandFactory(SwerveSubsystem subsystem) {
    this.subsystem = subsystem;
    rotationSources = new ArrayList<>(1);
  }

  public Command applyRequest(Supplier<LegacySwerveRequest> requestSupplier) {
    if (subsystem == null) return Commands.none();
    return subsystem
        .run(() -> subsystem.setControl(requestSupplier.get()))
        .withName("applyRequest")
        .asProxy();
  }

  // ---------- Getters ----------

  public Translation2d currentRobotPosition() {
    if (subsystem == null) return new Translation2d();
    return subsystem.getState().Pose.getTranslation();
  }

  // ---------- Commands ----------

  /**
   * Request the robot to point in a specified direction. Any non zero rotation demand will result
   * in the command canceling.
   *
   * @param angleToPoint the angle to point in degrees
   * @param input the X-Y speeds
   * @return A command that will point in the specified direction until interupted
   */
  public Command pointInDirection(Rotation2d angleToPoint, Supplier<DriveRequest> input) {
    if (subsystem == null) return Commands.none();
    return pointDrive(() -> angleToPoint.getDegrees(), input).withName("Pointing In Direction");
  }

  /**
   * Points toward a location on the field any non zero rotation demand will result in the command
   * canceling.
   *
   * @param target the location to point at in meters
   * @param input X-Y speeds
   * @return A command that will point at the specified location until interupted
   */
  public Command pointAtLocation(final Translation2d target, final Supplier<DriveRequest> input) {
    if (subsystem == null) return Commands.none();
    final DoubleSupplier getAngleToTarget =
        () -> HowdyMath.getAngleToTarget(currentRobotPosition(), target).getDegrees();
    return pointDrive(getAngleToTarget, input).withName("Pointing at location").asProxy();
  }

  /**
   * Makes the robot point in a specfic direction. While still being able to move in the other 2
   * axis any non zero rotation demand will result in the command canceling.
   *
   * @param targetAngleProvider - the target angle provider to point from robot 0 in degreees
   * @param input the human X-Y request
   * @return A command that will point in the requested direction until interupted
   */
  public Command pointDrive(
      final DoubleSupplier targetAngleProvider, final Supplier<DriveRequest> input) {
    if (subsystem == null) return Commands.none();
    final ProfiledPIDController pid =
        new ProfiledPIDController(
            Constants.SwerveDriveConstants.TURN_kP,
            0,
            Constants.SwerveDriveConstants.TURN_kD,
            new TrapezoidProfile.Constraints(
                Constants.SwerveDriveConstants.MAX_AUTO_TURN,
                Constants.SwerveDriveConstants.MAX_AUTO_ACCERLATION));
    pid.enableContinuousInput(0, 360);
    final Runnable init =
        () -> {
          pid.reset(subsystem.getState().Pose.getRotation().getDegrees());
        };

    final Runnable exec =
        () -> {
          DriveRequest in = input.get();
          pid.setGoal(targetAngleProvider.getAsDouble());
          double alpha =
              Math.toRadians(pid.calculate(subsystem.getState().Pose.getRotation().getDegrees()));
          SmartDashboard.putNumber("error", pid.getPositionError());
          SmartDashboard.putNumber("alpha", alpha);
          subsystem.setControl(
              new LegacySwerveRequest.FieldCentric()
                  .withRotationalRate(alpha)
                  .withVelocityX(in.xSpeed() * SwerveSubsystem.maxSpeed)
                  .withVelocityY(in.ySpeed() * SwerveSubsystem.maxSpeed));
          if (in.alpha() != 0) {
            subsystem.getCurrentCommand().cancel();
          }
        };

    final Command command =
        new FunctionalCommand(init, exec, (interupt) -> {}, () -> false, subsystem);
    return command.withName("pointDrive").asProxy();
  }

  /**
   * Rotate to rotation with error from the target rotation being fed in.
   *
   * @param err The error from the target target rotation
   * @param input X-Y speed demands, non-zero alpha will cancel subsystem demand
   * @return A command that rotates to attempts to zero the error given.
   */
  public Command rotationDrive(final Supplier<Rotation2d> err, final Supplier<DriveRequest> input) {
    if (subsystem == null) return Commands.none();
    final PIDController pid =
        new PIDController(
            Constants.SwerveDriveConstants.TURN_kP, 0, Constants.SwerveDriveConstants.TURN_kD);
    pid.enableContinuousInput(0, 360);
    pid.setSetpoint(0);
    final Runnable command =
        () -> {
          DriveRequest in = input.get();
          double alpha = Math.toRadians(pid.calculate(err.get().getDegrees()));
          subsystem.setControl(
              new LegacySwerveRequest.FieldCentric()
                  .withRotationalRate(alpha)
                  .withVelocityX(in.xSpeed() * SwerveSubsystem.maxSpeed)
                  .withVelocityY(in.ySpeed() * SwerveSubsystem.maxSpeed));
          if (in.alpha() != 0) {
            subsystem.getCurrentCommand().cancel();
          }
        };

    return subsystem.run(command).finallyDo(() -> pid.close()).withName("rotationDrive").asProxy();
  }

  public Command robotOrientedDrive(final Supplier<DriveRequest> requestSupplier) {
    if (subsystem == null) return Commands.none();

    final Runnable command =
        () -> {
          DriveRequest driveRequest = requestSupplier.get();
          LegacySwerveRequest swerveRequest =
              new LegacySwerveRequest.RobotCentric()
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                  .withVelocityX(driveRequest.xSpeed() * SwerveSubsystem.maxSpeed)
                  .withVelocityY(driveRequest.ySpeed() * SwerveSubsystem.maxSpeed)
                  .withRotationalRate(driveRequest.alpha() * SwerveSubsystem.maxAngularRate);
          subsystem.setControl(swerveRequest);
        };

    return subsystem.run(command).withName("robotOrientedDrive").asProxy();
  }

  public Command fieldOrientedDrive(final Supplier<DriveRequest> requestSupplier) {
    if (subsystem == null) return Commands.none();
    final Command command =
        subsystem
            .run(
                () -> {
                  DriveRequest driveRequest = requestSupplier.get();
                  LegacySwerveRequest swerveRequest =
                      new LegacySwerveRequest.FieldCentric()
                          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                          .withVelocityX(driveRequest.xSpeed() * SwerveSubsystem.maxSpeed)
                          .withVelocityY(driveRequest.ySpeed() * SwerveSubsystem.maxSpeed)
                          .withRotationalRate(
                              driveRequest.alpha() * SwerveSubsystem.maxAngularRate);
                  subsystem.setControl(swerveRequest);
                })
            .withName("fieldOrientedDrive")
            .asProxy();
    return command;
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(
        Commands.parallel(subsystem.runOnce(() -> {}), defaultCommand)
            .withName(defaultCommand.getName()));
  }

  public Command zeroDriveTrain() {
    if (subsystem == null) return Commands.none();

    return subsystem
        .runOnce(
            () -> {
              for (RotationSource rs : rotationSources) {
                rs.zero();
              }
              subsystem.seedFieldRelative(
                  new Pose2d(
                      subsystem.getState().Pose.getTranslation(), Rotation2d.fromDegrees(180)));
            })
        .withName("zeroDriveTrain")
        .asProxy();
  }

  public Command assistedDriving(Supplier<DriveRequest> requestSupplier, RobotStateManager RSM) {
    /*
     * 4 Assisted Drives
     *
     * 1. Auto target speaker
     * 2. Auto target amp
     * 3. Point at source
     *
     * Decision flow:
     * is on near side?
     * yes{
     *  is amp mode?
     *  yes{
     *   target amp
     *  }
     *  no{
     *   target speaker
     *  }
     * }
     * no{
     *  is amp mode?
     *  yes{
     *   no op
     *  }
     *  no{
     *   target source
     *  }
     * }
     */

    final BooleanSupplier onNearSide =
        isOnNearSide(() -> RSM.getAllianceColor() == AllianceColor.RED);
    final BooleanSupplier isAmpMode = RSM.isAmpSupplier();

    final Command assistDriver =
        Commands.either(
            Commands.either(
                autoTargetAmp(requestSupplier, RSM),
                autoTargetSpeaker(requestSupplier, RSM),
                isAmpMode),
            Commands.either(Commands.none(), autoTargetSource(requestSupplier, RSM), isAmpMode),
            onNearSide);

    return assistDriver.withName("assistedDriving");
  }

  private BooleanSupplier isOnNearSide(BooleanSupplier isRed) {
    return () -> {
      boolean isNear =
          subsystem.getState().Pose.getX() > FieldConstants.CENTERLINE_X_APPROX
              ^ !isRed.getAsBoolean();
      SmartDashboard.putBoolean("nearSide", isNear);
      return isNear;
    };
  }

  public Command autoTargetSource(Supplier<DriveRequest> request, RobotStateManager RSM) {
    return Commands.either(
            pointInDirection(DriverConstants.RED_SOURCE_ROTATION, request),
            pointInDirection(DriverConstants.BLUE_SOURCE_ROTATION, request),
            () -> RSM.getAllianceColor() == AllianceColor.RED)
        .withName("Target Source")
        .asProxy();
  }

  public Command autoTargetSpeaker(Supplier<DriveRequest> request, RobotStateManager RSM) {
    return Commands.either(
            pointAtLocation(FieldConstants.RED_SPEAKER, request),
            pointAtLocation(FieldConstants.BLUE_SPEAKER, request),
            () -> RSM.getAllianceColor() == AllianceColor.RED)
        .withName("Target Speaker")
        .asProxy();
  }

  public Command autoTargetAmp(Supplier<DriveRequest> request, RobotStateManager RSM) {
    return Commands.either(
            pointInDirection(DriverConstants.RED_AMP_ROTATION, request),
            pointInDirection(DriverConstants.BLUE_AMP_ROTATION, request),
            () -> RSM.getAllianceColor() == AllianceColor.RED)
        .withName("Target Amp")
        .asProxy();
  }

  public RotationSource createRotationSource(
      XboxController controller, SwerveSubsystem drivetrain) {
    final RotationSource RS = new RotationSource(controller, drivetrain);
    rotationSources.add(RS);
    return RS;
  }

  public Command[] getCommands() {
    ArrayList<Command> cmds = new ArrayList<Command>();
    cmds.add(autoTargetAmp(() -> new DriveRequest(0, 0, 0), new RobotStateManager()));
    cmds.add(autoTargetSpeaker(() -> new DriveRequest(0, 0, 0), new RobotStateManager()));
    cmds.add(assistedDriving(() -> new DriveRequest(0, 0, 0), new RobotStateManager()));
    cmds.add(zeroDriveTrain());
    cmds.add(fieldOrientedDrive(() -> new DriveRequest(0, 0, 0)));
    cmds.add(robotOrientedDrive(() -> new DriveRequest(0, 0, 0)));
    cmds.add(applyRequest(() -> new LegacySwerveRequest.Idle()));
    cmds.add(pointAtLocation(new Translation2d(), () -> new DriveRequest(0, 0, 0)));
    cmds.add(pointInDirection(new Rotation2d(), () -> new DriveRequest(0, 0, 0)));
    cmds.add(pointDrive(() -> 0.0, () -> new DriveRequest(0, 0, 0)));
    cmds.add(this.rotationDrive(() -> new Rotation2d(), () -> new DriveRequest(0, 0, 0)));
    cmds.add(this.autoTargetSource(() -> new DriveRequest(0, 0, 0), new RobotStateManager()));
    return cmds.toArray(new Command[cmds.size()]);
  }
}
