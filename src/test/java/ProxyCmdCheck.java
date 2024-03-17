import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
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
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem.DriveRequest;
import frc.robot.subsystems.trapElvSubsystem.TrapElvCommandFactory;
import frc.robot.subsystems.trapElvSubsystem.TrapElvSubsystem;
import frc.robot.subsystems.triggerSubsystem.TriggerCommandFactory;
import frc.robot.subsystems.triggerSubsystem.TriggerSubsystem;
import frc.robot.subsystems.turretSubsystem.TurretCommandFactory;
import frc.robot.subsystems.turretSubsystem.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ProxyCmdCheck {

  interface SupplierRotation2d extends Supplier<Rotation2d> {}

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
        ArrayList<Object[]> params = new ArrayList<Object[]>();
        params.add(new Object[types.length]);

        for (int i = 0; i < types.length; i++) {
          switch (types[i].getName()) {
            case "int":
            case "Integer":
              for (Object[] p : params) {
                p[i] = Integer.valueOf(0);
              }
              break;
            case "double":
            case "Double":
              for (Object[] p : params) {
                p[i] = Double.valueOf(0);
              }
              break;
            case "String":
              for (Object[] p : params) {
                p[i] = "";
              }
              break;
            case "boolean":
            case "Boolean":
              for (Object[] p : params) {
                p[i] = Boolean.valueOf(false);
              }
              break;
            case "float":
            case "Float":
              for (Object[] p : params) {
                p[i] = Float.valueOf(0);
              }
              break;
            case "java.util.function.BooleanSupplier":
              for (Object[] p : params) {
                p[i] = new Trigger(() -> false);
              }
              break;
            case "edu.wpi.first.math.geometry.Translation2d":
              for (Object[] p : params) {
                p[i] = new Translation2d();
              }
              break;

            case "edu.wpi.first.math.geometry.Rotation2d":
              for (Object[] p : params) {
                p[i] = new Rotation2d();
              }
              break;

            case "java.util.function.DoubleSupplier":
              for (Object[] p : params) {
                p[i] =
                    (DoubleSupplier)
                        () -> {
                          return 0.0;
                        };
              }
              break;
            case "java.util.function.Supplier":
              // Type erasure means it is impossible to know what type of supplier it will be
              // Naive solution to try all possibilites until it doesn't throw
              int saveSize = params.size();
              for (int j = 0; j < saveSize; j++) {
                params.add(params.get(j).clone());
                params.get(params.size() - 1)[i] =
                    (Supplier<Rotation2d>)
                        () -> {
                          return new Rotation2d();
                        };
                params.add(params.get(j).clone());
                params.get(params.size() - 1)[i] =
                    (Supplier<DriveRequest>)
                        () -> {
                          return new DriveRequest(0, 0, 0);
                        };
                params.add(params.get(j).clone());
                params.get(params.size() - 1)[i] =
                    (Supplier<SwerveRequest>)
                        () -> {
                          return new SwerveRequest.Idle();
                        };
              }
              break;
            case "frc.robot.stateManagement.RobotStateManager":
              for (Object[] p : params) {
                p[i] = new RobotStateManager();
              }
              break;

            case "frc.robot.stateManagement.PlacementMode":
              for (Object[] p : params) {
                p[i] = PlacementMode.AMP;
              }
              break;

            default:
              throw new IllegalArgumentException(
                  "Unsupported Command Factory parameter type " + types[i].getName());
          }
        }
        Command cmd = null;
        for (Object[] p : params) {
          try {
            cmd = (Command) m.invoke(factory, p);
            break;
          } catch (Exception e) {
            // TODO Auto-generated catch block
            System.out.println(e.getCause().toString() + " on " + m.getName());
          }
        }
        if (cmd == null) {
          assertEquals(false, true, "No valid parameter sets found");

        } else {

          assertEquals(
              new HashSet<Subsystem>(),
              cmd.getRequirements(),
              m.getName() + " has " + factory.getClass().getName() + " requirement");
        }
      }
    }
  }
}
