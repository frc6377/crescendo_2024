package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
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
  private boolean checkAllianceColor = true;

  // End Game
  private boolean isEndGame = false;
  private final BooleanTopic endGameTopic =
      NetworkTableInstance.getDefault().getBooleanTopic("END_GAME");
  private final BooleanPublisher endGamePublisher = endGameTopic.publish();

  // Note State
  private final IntegerTopic noteStateTopic =
      NetworkTableInstance.getDefault().getIntegerTopic("NOTE_STATE");
  private final IntegerPublisher noteStatePublisher = noteStateTopic.publish();
  private final IntegerSubscriber noteStateSubscriber =
      noteStateTopic.subscribe(NoteState.NO_TE.getAsInt()); // Default to NO_TE

  // Placing Mode
  private final IntegerTopic placementModeTopic =
      NetworkTableInstance.getDefault().getIntegerTopic("PLACMENT_MODE");
  private final IntegerPublisher placementModePublisher = placementModeTopic.publish();
  private final IntegerSubscriber placementModeSubscriber =
      placementModeTopic.subscribe(PlacementMode.SPEAKER.getAsInt()); // Default to Speaker

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
    if (!isEndGame) {
      double matchTimer = DriverStation.getMatchTime();
      boolean isTeleopEnabled = DriverStation.isTeleopEnabled();
      if (matchTimer < 20 && isTeleopEnabled) {
        isEndGame = true;
        endGamePublisher.accept(true);
      }
    }
  }

  // Note State
  public void setNoteState(NoteState noteState) {
    noteStatePublisher.accept(noteState.getAsInt());
  }

  // Placement Mode
  public void setPlacementMode(PlacementMode placementMode) {
    placementModePublisher.accept(placementMode.getAsInt());
  }
}
