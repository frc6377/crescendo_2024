package frc.robot.subsystems.climberSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climberSubsystem.ClimberSubsystem.DifferentialDemand;
import frc.robot.subsystems.climberSubsystem.ClimberSubsystem.Position;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class ClimberCommandFactory {
  private final ClimberSubsystem subsystem;
  private final Runnable noop = () -> {};

  public ClimberCommandFactory(ClimberSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command raise() {
    if (subsystem == null) return new InstantCommand();
    Timer minTime = new Timer();
    return new FunctionalCommand(
        () -> {
          subsystem.applyCurrentDemand(10);
          minTime.start();
        },
        noop,
        (interupt) -> {
          if (interupt) return;
          subsystem.applyPercent(0);
        },
        () -> {
          boolean vel = subsystem.getVelocity().isZero(0.5);
          boolean cur = subsystem.getCurrent().greater(0.1);
          System.err.println("done" + vel + "," + cur);
          return vel && minTime.hasElapsed(ClimberConstants.MIN_RAISE_TIME_SEC);
        },
        subsystem);
  }

  public Command clip() {
    if (subsystem == null) return new InstantCommand();
    return subsystem.run(() -> subsystem.applyCurrentDemand(ClimberConstants.CLIP_CURRENT));
  }

  boolean right = true;
  boolean left = true;

  public Command climb() {
    if (subsystem == null) return new InstantCommand();

    Runnable init = () -> {};
    Runnable exec =
        () -> {
          Position pos = subsystem.getPosition();
          double rightDemand;
          double leftDemand;
          if (right && pos.right() > ClimberConstants.CLIMB_POSITION) {
            rightDemand = ClimberConstants.CLIMB_PERCENT;
          } else {
            right = false;
            rightDemand = 0;
          }
          if (left && pos.left() > ClimberConstants.CLIMB_POSITION) {
            leftDemand = ClimberConstants.CLIMB_PERCENT;
          } else {
            left = false;
            leftDemand = 0;
          }
          subsystem.applyDiffertialPercent(new DifferentialDemand(leftDemand, rightDemand));
        };
    Consumer<Boolean> end = (interupt) -> {};
    BooleanSupplier isFinished =
        () -> subsystem.getPosition().isLessThen(ClimberConstants.CLIMB_POSITION);

    return new FunctionalCommand(init, exec, end, isFinished, subsystem);
  }
}
