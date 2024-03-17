import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.stateManagement.PlacementMode;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.climberSubsystem.ClimberCommandFactory;
import frc.robot.subsystems.climberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.intakeSubsystem.IntakeCommandFactory;
import frc.robot.subsystems.intakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.shooterSubsystem.ShooterCommandFactory;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.swerveSubsystem.SwerveCommandFactory;
import frc.robot.subsystems.trapElvSubsystem.TrapElvCommandFactory;
import frc.robot.subsystems.trapElvSubsystem.TrapElvSubsystem;
import frc.robot.subsystems.triggerSubsystem.TriggerCommandFactory;
import frc.robot.subsystems.triggerSubsystem.TriggerSubsystem;
import frc.robot.subsystems.turretSubsystem.TurretCommandFactory;
import frc.robot.subsystems.turretSubsystem.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.lang.reflect.Method;
import java.util.HashSet;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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

  @Test
  public void checkSwerveCmdsAreProxy() {
    checkAllCmdFactoriesAreProxy(
        new SwerveCommandFactory(new DynamicRobotConfig().getTunerConstants().drivetrain));
  }

  @Test
  public void checkIntakeCmdsAreProxy() {
    checkAllCmdFactoriesAreProxy(new IntakeCommandFactory(new IntakeSubsystem()));
  }

  private void checkAllCmdFactoriesAreProxy(Object factory) {
    for (Method m : factory.getClass().getMethods()) {
      if (m.getReturnType() == Command.class) {
        Class<?> types[] = m.getParameterTypes();
        Object[] params = new Object[types.length];
        Object[] p = m.getParameters();

        for (int i = 0; i < types.length; i++) {
          switch (types[i].getName()) {
            case "int":
            case "Integer":
              params[i] = Integer.valueOf(0);
              break;
            case "double":
            case "Double":
              params[i] = Double.valueOf(0);
              break;
            case "String":
              params[i] = "";
              break;
            case "boolean":
            case "Boolean":
              params[i] = Boolean.valueOf(false);
              break;
            case "float":
            case "Float":
              params[i] = Float.valueOf(0);
              break;
            case "java.util.function.BooleanSupplier":
              params[i] = new Trigger(() -> false);
              break;
            case "edu.wpi.first.math.geometry.Translation2d":
              params[i] = new Translation2d();
              break;

            case "edu.wpi.first.math.geometry.Rotation2d":
              params[i] = new Rotation2d();
              break;

            case "java.util.function.DoubleSupplier":
              params[i] =
                  (DoubleSupplier)
                      () -> {
                        return 0.0;
                      };
              break;
            case "java.util.function.Supplier":
              // Since the Commands are compiled and not run, the Supplier generic can be anything

              params[i] =
                  (Supplier<Double>)
                      () -> {
                        return 0.0;
                      };
              break;
            case "frc.robot.stateManagement.RobotStateManager":
              params[i] = new RobotStateManager();
              break;

            case "frc.robot.stateManagement.PlacementMode":
              params[i] = PlacementMode.AMP;
              break;

            default:
              throw new IllegalArgumentException(
                  "Unsupported Command Factory parameter type " + types[i].getName());
          }
        }
        Command cmd = null;

        try {
          System.out.println("Testing " + m.getName());
          cmd = (Command) m.invoke(factory, params);
          assertEquals(
              new HashSet<Subsystem>(),
              cmd.getRequirements(),
              m.getName() + " has " + factory.getClass().getName() + " requirement");
          System.out.println(m.getName() + " Passed");

        } catch (Exception e) {
          // TODO Auto-generated catch block
          System.out.println(e.getMessage() + " on " + m.getName());
        }
      }
    }
  }
}
