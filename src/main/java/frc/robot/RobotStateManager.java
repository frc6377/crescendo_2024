package frc.robot;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class RobotStateManager extends SubsystemBase {
  // Alliance Color
  private final IntegerTopic allianceTopic =
      NetworkTableInstance.getDefault().getIntegerTopic("ALLIANCE");
  private final IntegerPublisher alliancePublisher = allianceTopic.publish();
  private final IntegerSubscriber allianceSubscriber = allianceTopic.subscribe(0);
  private boolean checkAllianceColor = true;

  public RobotStateManager() {}

  @Override
  public void periodic() {
    if (checkAllianceColor) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        checkAllianceColor = false;
        alliancePublisher.accept(alliance.get().equals(Alliance.Red) ? 10 : 20);
      }
    }
  }
}
