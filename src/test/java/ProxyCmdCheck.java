import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.climberSubsystem.ClimberCommandFactory;
import frc.robot.subsystems.climberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.shooterSubsystem.ShooterCommandFactory;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.trapElvSubsystem.TrapElvCommandFactory;
import frc.robot.subsystems.trapElvSubsystem.TrapElvSubsystem;
import frc.robot.subsystems.triggerSubsystem.TriggerCommandFactory;
import frc.robot.subsystems.triggerSubsystem.TriggerSubsystem;
import frc.robot.subsystems.turretSubsystem.TurretCommandFactory;
import frc.robot.subsystems.turretSubsystem.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.lang.reflect.InvocationTargetException;
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
    checkAllCmdFactoriesAreProxy(new ClimberCommandFactory(new ClimberSubsystem()));
  }

  @Test
  public void checkShooterCmdsAreProxy() {
    checkAllCmdFactoriesAreProxy(new ShooterCommandFactory(new ShooterSubsystem()));
  }

  @Test
  public void checkTrapElvCmdsAreProxy() {
    checkAllCmdFactoriesAreProxy(new TrapElvCommandFactory(new TrapElvSubsystem()));
  }

  @Test
  public void checkTriggerCmdsAreProxy() {
    checkAllCmdFactoriesAreProxy(new TriggerCommandFactory(new TriggerSubsystem()));
  }

  @Test
  public void checkTurretCmdsAreProxy() {
    checkAllCmdFactoriesAreProxy(
        new TurretCommandFactory(
            new TurretSubsystem(new RobotStateManager(), new VisionSubsystem() {})));
  }

  private void checkAllCmdFactoriesAreProxy(Object factory) {
    for (Method m : factory.getClass().getMethods()) {
      if (m.getReturnType() == Command.class) {
        try {
          Command cmd = (Command) m.invoke(factory); // TODO need to generalize command parameters
          assertEquals(
              new HashSet<Subsystem>(),
              cmd.getRequirements(),
              m.getName() + " has " + factory.getClass().getName() + " requirement");
        } catch (IllegalAccessException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        } catch (IllegalArgumentException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        } catch (InvocationTargetException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    }
  }
}
