import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.climberSubsystem.ClimberCommandFactory;
import frc.robot.subsystems.climberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.intakeSubsystem.IntakeCommandFactory;
import frc.robot.subsystems.intakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.shooterSubsystem.ShooterCommandFactory;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.swerveSubsystem.SwerveCommandFactory;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem;
import frc.robot.subsystems.trapElvSubsystem.TrapElvCommandFactory;
import frc.robot.subsystems.trapElvSubsystem.TrapElvSubsystem;
import frc.robot.subsystems.triggerSubsystem.TriggerCommandFactory;
import frc.robot.subsystems.triggerSubsystem.TriggerSubsystem;
import frc.robot.subsystems.turretSubsystem.TurretCommandFactory;
import frc.robot.subsystems.turretSubsystem.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.lang.reflect.Method;
import java.util.HashSet;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

public class CommandFactoryChecks {

  @BeforeEach
  public void setup() {}

  @AfterEach
  public void cleanupTests() {}

  @Test
  public void checkClimbCmds() {
    ClimberSubsystem sub = new ClimberSubsystem();
    ClimberCommandFactory factory = new ClimberCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands(), sub);

    factory = new ClimberCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkSwerveCmds() {
    SwerveSubsystem sub = new DynamicRobotConfig().getTunerConstants().drivetrain;
    SwerveCommandFactory factory = new SwerveCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands(), sub);

    factory = new SwerveCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkShootCmds() {
    ShooterSubsystem sub = new ShooterSubsystem();
    ShooterCommandFactory factory = new ShooterCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands(), sub);

    factory = new ShooterCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkTrapElvCmds() {
    TrapElvSubsystem sub = new TrapElvSubsystem();
    TrapElvCommandFactory factory = new TrapElvCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands(), sub);

    factory = new TrapElvCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkIntakeCmds() {
    IntakeSubsystem sub = new IntakeSubsystem();
    IntakeCommandFactory factory = new IntakeCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands(), sub);

    factory = new IntakeCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkTriggerCmds() {
    TriggerSubsystem sub = new TriggerSubsystem();
    TriggerCommandFactory factory = new TriggerCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands(), sub);

    factory = new TriggerCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkTurretCmds() {
    TurretSubsystem sub = new TurretSubsystem(new RobotStateManager(), new VisionSubsystem() {});
    TurretCommandFactory factory = new TurretCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands(), sub);

    factory = new TurretCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  // TODO: only checks Command factor methods, other methods remain unchecked
  private void checkCmdNullSafety(Runnable getCommands) {
    try {
      getCommands.run();
    } catch (NullPointerException e) {
      assertEquals(true, false, "Caught NullPointerException when subsystem is disabled");
    }
  }

  private void noDuplicatesInGetCommands(Command[] cmds) {
    for (int i = 0; i < cmds.length; i++) {
      String name = cmds[i].getName();
      for (int j = i + 1; j < cmds.length; j++) {
        assertNotEquals(
            name, cmds[j].getName(), "Duplicate Command \"" + name + "\" in getCommands list");
      }
    }
  }

  private void checkAllCmdFactoriesAreProxy(Object factory, Command[] cmds, Subsystem sub) {
    int numCommands = 0;
    for (Method m : factory.getClass().getMethods()) {
      if (m.getReturnType() == Command.class) {
        numCommands++;
      }
    }
    noDuplicatesInGetCommands(cmds);
    assertEquals(
        cmds.length,
        numCommands,
        sub.getName() + " getCommands() list size mismatch with number of Command factory methods");

    for (Command cmd : cmds) {
      // Verify the compiled Command doesn't require any subsystem
      assertEquals(
          new HashSet<Subsystem>(),
          cmd.getRequirements(),
          "Non-proxied public Command factory detected: " + cmd.getName());

      try (MockedStatic<RobotState> robotMock = Mockito.mockStatic(RobotState.class)) {
        // force robot enabled so the CommandScheduler can schedule Commands.
        robotMock.when(() -> RobotState.isDisabled()).thenReturn(false);
        CommandScheduler.getInstance().enable();

        // Schedule the command and verify is requires the subsystem underneath the ProxyCommand
        cmd.schedule();
        assertNotEquals(
            CommandScheduler.getInstance().requiring(sub),
            null,
            cmd.getName() + " missing subsystem requirement");
        CommandScheduler.getInstance().cancelAll();
      }
    }
  }
}
