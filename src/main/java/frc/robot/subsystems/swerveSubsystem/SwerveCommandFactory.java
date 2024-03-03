package frc.robot.subsystems.swerveSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem.DriveRequest;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommandFactory {
  private final SwerveSubsystem subsystem;

  public SwerveCommandFactory(SwerveSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    if (subsystem == null) return new InstantCommand();
    return subsystem
        .run(() -> subsystem.setControl(requestSupplier.get()))
        .withName("Request Supplier");
  }

  /**
   * Request the robot to point in a specified direction. Any non zero rotation demand will result
   * in the command canceling.
   *
   * @param angleToPoint the angle to point in degrees
   * @param input the X-Y speeds
   * @return A command that will point in the specified direction until interupted
   */
  public Command pointInDirection(Rotation2d angleToPoint, Supplier<DriveRequest> input) {
    if (subsystem == null) return new InstantCommand();
    return pointDrive(() -> angleToPoint.getDegrees(), input).withName("Pointing in direction");
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
    if (subsystem == null) return new InstantCommand();
    final DoubleSupplier getAngleToTarget =
        () -> {
          Translation2d delta = subsystem.getState().Pose.getTranslation().minus(target);
          return new Rotation2d(delta.getX(), delta.getY()).getDegrees();
        };
    return pointDrive(getAngleToTarget, input).withName("Pointing at location");
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
    if (subsystem == null) return new InstantCommand();
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
          subsystem.setControl(
              new SwerveRequest.FieldCentric()
                  .withRotationalRate(alpha)
                  .withVelocityX(in.xSpeed() * SwerveSubsystem.maxSpeed)
                  .withVelocityY(in.ySpeed() * SwerveSubsystem.maxSpeed));
          if (in.alpha() != 0) {
            subsystem.getCurrentCommand().cancel();
          }
        };

    final Command command =
        new FunctionalCommand(init, exec, (interupt) -> {}, () -> false, subsystem);
    return command.withName("Point Drive");
  }

  /**
   * Rotate to rotation with error from the target rotation being fed in.
   *
   * @param err The error from the target target rotation
   * @param input X-Y speed demands, non-zero alpha will cancel subsystem demand
   * @return A command that rotates to attempts to zero the error given.
   */
  public Command rotationDrive(final Supplier<Rotation2d> err, final Supplier<DriveRequest> input) {
    if (subsystem == null) return new InstantCommand();
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
              new SwerveRequest.FieldCentric()
                  .withRotationalRate(alpha)
                  .withVelocityX(in.xSpeed() * SwerveSubsystem.maxSpeed)
                  .withVelocityY(in.ySpeed() * SwerveSubsystem.maxSpeed));
          if (in.alpha() != 0) {
            subsystem.getCurrentCommand().cancel();
          }
        };

    return subsystem.run(command).finallyDo(() -> pid.close()).withName("Rotation Drive");
  }

  public Command robotOrientedDrive(final Supplier<DriveRequest> requestSupplier) {
    if (subsystem == null) return new InstantCommand();
    final Runnable command =
        () -> {
          DriveRequest driveRequest = requestSupplier.get();
          SwerveRequest swerveRequest =
              new SwerveRequest.RobotCentric()
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                  .withVelocityX(driveRequest.xSpeed() * SwerveSubsystem.maxSpeed)
                  .withVelocityY(driveRequest.ySpeed() * SwerveSubsystem.maxSpeed)
                  .withRotationalRate(driveRequest.alpha() * SwerveSubsystem.maxAngularRate);
          subsystem.setControl(swerveRequest);
        };

    return subsystem.run(command);
  }

  public Command fieldOrientedDrive(final Supplier<DriveRequest> requestSupplier) {
    if (subsystem == null) return new InstantCommand();
    final Runnable command =
        () -> {
          DriveRequest driveRequest = requestSupplier.get();
          SwerveRequest swerveRequest =
              new SwerveRequest.FieldCentric()
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                  .withVelocityX(driveRequest.xSpeed() * SwerveSubsystem.maxSpeed)
                  .withVelocityY(driveRequest.ySpeed() * SwerveSubsystem.maxSpeed)
                  .withRotationalRate(driveRequest.alpha() * SwerveSubsystem.maxAngularRate);
          subsystem.setControl(swerveRequest);
        };

    return subsystem.run(command);
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }

  public Command zeroDriveTrain() {
    if (subsystem == null) return new InstantCommand();
    return subsystem.runOnce(
        () ->
            subsystem.seedFieldRelative(
                new Pose2d(
                    subsystem.getState().Pose.getTranslation(), Rotation2d.fromDegrees(180))));
  }

  public Command assistedDriving(){
    /*
     * 4 Assisted Drives
     * 
     * 1. Auto target speaker
     * 2. Auto target amp
     * 3. Point at source
     * 
     * Decision flow:
     * Is amp mode?
     * Yes:
     *  Is on near side of Field?
     *  Yes:
     *   Auto target amp
     *   No Op
     * No:
     *  Is on near side of Field?
     *  Yes:
     *    Target speaker
     *   No:
     *    Target source
     */

    Pose2d robotPosition;

    return new InstantCommand();
  }
}
