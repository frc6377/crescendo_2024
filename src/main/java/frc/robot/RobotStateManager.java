package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotStateManager extends SubsystemBase {
  /*public final IntegerTopic gamePieceModeTopic =
      NetworkTableInstance.getDefault().getIntegerTopic("GAME_PIECE_MODE");
  public final IntegerPublisher gamePieceModePublisher = gamePieceModeTopic.publish();
  public final IntegerSubscriber gamePieceModeSubscriber = gamePieceModeTopic.subscribe(10);*/

  // Left old GamePieceMode code commented for example
  public RobotStateManager(/*GamePieceMode gamePieceMode*/ ) {
    // this.gamePieceModePublisher.accept(gamePieceMode.getAsInt());
  }

  /*public void SwitchCubeCone() {
    GamePieceMode currentGamePieceMode =
        GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get());
    gamePieceModePublisher.accept(
        (currentGamePieceMode.isCube() ? GamePieceMode.CONE : GamePieceMode.CUBE).getAsInt());
  }*/

  /*public GamePieceMode getGamePieceMode() {
    return GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get());
  }*/
}
