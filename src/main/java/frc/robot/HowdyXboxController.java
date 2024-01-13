package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class HowdyXboxController extends CommandXboxController {
  private final XboxController rumbleXboxController;

  /**
   * Creates a Command Xbox Controller with rumble support added
   *
   * @param port The controller port
   */
  public HowdyXboxController(int port) {
    super(port);
    this.rumbleXboxController = new XboxController(port);
  }

  public void setRumble(double rumbleIntensity) {
    rumbleXboxController.setRumble(RumbleType.kBothRumble, rumbleIntensity);
  }
}
