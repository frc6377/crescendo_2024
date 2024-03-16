package frc.robot.utilities;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooterSubsystem.ShooterCommandFactory;
import java.util.function.BooleanSupplier;

public class AutoManager {
  SanAnStatus sanAnInstance = new SanAnStatus();

  public AutoManager(ShooterCommandFactory shooterSubsystem) {
    waitForShooterLoad(shooterSubsystem.getBeamBreak());
    sanAnInstance.register();
  }

  private void waitForShooterLoad(BooleanSupplier loadDetector) {
    NamedCommands.registerCommand("Wait until Loaded", Commands.waitUntil(loadDetector));
  }

  private class SanAnStatus {
    private boolean inFiringLoction = false;

    public void register() {
      NamedCommands.registerCommand(
          "StartSanAn2Forward", new InstantCommand(() -> inFiringLoction = false));
      NamedCommands.registerCommand(
          "atSanAnFiringLocation", new InstantCommand(() -> inFiringLoction = true));
      NamedCommands.registerCommand(
          "WaitForSanAnFiringLocation", new WaitUntilCommand(() -> inFiringLoction));
    }
  }
}
