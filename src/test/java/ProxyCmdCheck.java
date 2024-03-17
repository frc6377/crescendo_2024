import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.wpilibj2.command.Command;
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

public class ProxyCmdCheck {

  @BeforeEach
  public void setup() {}

  @AfterEach
  public void cleanupTests() {}

  @Test
  public void checkClimbCmdsAreProxy() {
    ClimberSubsystem sub = new ClimberSubsystem();
    ClimberCommandFactory factory = new ClimberCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands());

    factory = new ClimberCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkSwerveCmdsAreProxy() {
    SwerveSubsystem sub = new DynamicRobotConfig().getTunerConstants().drivetrain;
    SwerveCommandFactory factory = new SwerveCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands());

    factory = new SwerveCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkShootCmdsAreProxy() {
    ShooterSubsystem sub = new ShooterSubsystem();
    ShooterCommandFactory factory = new ShooterCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands());

    factory = new ShooterCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkTrapElvCmdsAreProxy() {
    TrapElvSubsystem sub = new TrapElvSubsystem();
    TrapElvCommandFactory factory = new TrapElvCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands());

    factory = new TrapElvCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkIntakeCmdsAreProxy() {
    IntakeSubsystem sub = new IntakeSubsystem();
    IntakeCommandFactory factory = new IntakeCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands());

    factory = new IntakeCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkTriggerCmdsAreProxy() {
    TriggerSubsystem sub = new TriggerSubsystem();
    TriggerCommandFactory factory = new TriggerCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands());

    factory = new TriggerCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

  @Test
  public void checkTurretCmdsAreProxy() {
    TurretSubsystem sub = new TurretSubsystem(new RobotStateManager(), new VisionSubsystem() {});
    TurretCommandFactory factory = new TurretCommandFactory(sub);
    checkAllCmdFactoriesAreProxy(factory, factory.getCommands());

    factory = new TurretCommandFactory(null);
    checkCmdNullSafety(factory::getCommands);
  }

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
        assertNotEquals(name, cmds[j].getName());
      }
    }
  }

  private void checkAllCmdFactoriesAreProxy(Object factory, Command[] cmds) {
    int numCommands = 0;
    for (Method m : factory.getClass().getMethods()) {
      if (m.getReturnType() == Command.class) {
        numCommands++;
      }
    }
    noDuplicatesInGetCommands(cmds);
    assertEquals(cmds.length, numCommands);
    for (Command cmd : cmds) {
      assertEquals(new HashSet<Subsystem>(), cmd.getRequirements());
    }
  }
}
