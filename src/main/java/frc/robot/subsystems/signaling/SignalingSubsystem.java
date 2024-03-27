package frc.robot.subsystems.signaling;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.signaling.patterns.FireFlyPattern;
import frc.robot.subsystems.signaling.patterns.PatternNode;
import frc.robot.subsystems.signaling.patterns.RainbowPattern;
import frc.robot.subsystems.signaling.patterns.TransFlag;
import java.util.function.Consumer;

public class SignalingSubsystem extends SubsystemBase {

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  private static final int numberOfLEDS = Constants.LED_COUNT;

  private int tick;
  private int patternTick;
  private final Timer rumbleTimer;
  private double rumbleEndTime = 0;

  private DisablePattern disablePattern = DisablePattern.getRandom();

  private final Consumer<Double> driverRumbleConsumer;

  private final RobotStateManager robotStateManager;
  private final Trigger isAmpModeTrigger;

  public SignalingSubsystem(
      final int ID,
      final Consumer<Double> driverRumbleConsumer,
      final RobotStateManager robotStateManager) {
    this.driverRumbleConsumer = driverRumbleConsumer;
    this.robotStateManager = robotStateManager;
    this.isAmpModeTrigger = robotStateManager.isAmpTrigger();

    tick = 0;
    patternTick = 0;
    rumbleTimer = new Timer();

    // Initialize LED Strip
    ledStrip = new AddressableLED(ID);
    ledBuffer = new AddressableLEDBuffer(numberOfLEDS);
    ledStrip.setLength(numberOfLEDS);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  @Override
  public void periodic() {
    // Update Light Pattern
    if (DriverStation.isDisabled()) updatePattern();

    isAmpModeTrigger.onFalse(runOnce(() -> setHalfStrip(RGB.RED)));
    isAmpModeTrigger.onTrue(runOnce(() -> setHalfStrip(RGB.BLUE)));

    // End Signaling
    if (rumbleTimer.get() > rumbleEndTime) {
      rumbleTimer.reset();
      driverRumbleConsumer.accept(0.0);
      resetLEDs();
    }
  }

  private void resetLEDs() {
    setFullStrip(RGB.BLACK);
  }

  private RGB getColorFromAlliance(AllianceColor alliance) {
    if (alliance == AllianceColor.RED) {
      return RGB.RED;
    } else if (alliance == AllianceColor.BLUE) {
      return RGB.BLUE;
    }
    return RGB.RED;
  }

  private void startSignal(final double time, final double intensity) {
    driverRumbleConsumer.accept(intensity);
    rumbleEndTime = time;
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  private void startSignal(final double time, final RGB rgb) {
    rumbleEndTime = time;
    setFullStrip(rgb);
    ledStrip.setData(ledBuffer);
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  private void startSignal(final double time, final double intensity, final RGB rgb) {
    driverRumbleConsumer.accept(intensity);
    rumbleEndTime = time;
    setFullStrip(rgb);
    ledStrip.setData(ledBuffer);
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  public void startAmpSignal() {
    startSignal(10, Constants.OperatorConstants.RUMBLE_STRENGTH, RGB.RED);
  }

  public void startShooterSignal() {
    startSignal(10, Constants.OperatorConstants.RUMBLE_STRENGTH, RGB.BLUE);
  }

  public void endSignal() {
    rumbleTimer.reset();
    driverRumbleConsumer.accept(0.0);
    resetLEDs();
  }

  private void setFullStrip(final RGB rgb) {
    setSection(rgb, 0, numberOfLEDS);
  }

  private void setHalfStrip(final RGB rgb) {
    resetLEDs();
    for (var i = 0; i < numberOfLEDS; i += 2) {
      ledBuffer.setRGB(
          i,
          (int) (rgb.red * Constants.LED_BRIGHTNESS),
          (int) (rgb.green * Constants.LED_BRIGHTNESS),
          (int) (rgb.blue * Constants.LED_BRIGHTNESS));
    }
    ledStrip.setData(ledBuffer);
  }

  private void setSection(final RGB rgb, final int startID, final int count) {
    for (var i = startID; i < startID + count; i++) {
      if (i > -1 && i < numberOfLEDS) {
        ledBuffer.setRGB(
            i,
            (int) (rgb.red * Constants.LED_BRIGHTNESS),
            (int) (rgb.green * Constants.LED_BRIGHTNESS),
            (int) (rgb.blue * Constants.LED_BRIGHTNESS));
      }
    }
  }

  private void updatePattern() {
    PatternNode[] pattern;
    int patternLength;

    tick++;
    if (tick > Constants.PATTERN_SPEED) {
      tick = 0;
      patternTick++;
    } else {
      return;
    }

    // DeltaBoard.putString("Disable Pattern", disablePattern.name());

    switch (disablePattern) {
        // case BI_FLAG:
        //   pattern = BIFlag.getColors(patternTick);
        //   break;
      case FIRE_FLY:
        pattern = FireFlyPattern.getPattern();
        patternLength = FireFlyPattern.getPatternLength();
        break;
      case RAINBOW:
        pattern = RainbowPattern.getPattern();
        patternLength = RainbowPattern.getPatternLength();
        break;
      case TRANS_FLAG:
        pattern = TransFlag.getPattern();
        patternLength = TransFlag.getPatternLength();
        break;
      default:
        pattern = RainbowPattern.getPattern();
        patternLength = RainbowPattern.getPatternLength();
        break;
    }
    int patternIndex = 0;
    patternTick %= patternLength;
    int LEDIndex = -patternTick - 1;
    while (LEDIndex < numberOfLEDS) {
      patternIndex %= pattern.length;

      PatternNode node = pattern[patternIndex];
      RGB c = pattern[patternIndex].color;

      setSection(c, LEDIndex, node.repeat);
      LEDIndex += node.repeat;
      patternIndex += 1;
    }
    ledStrip.setData(ledBuffer);
  }

  public void randomizePattern() {
    disablePattern = DisablePattern.getRandom();
  }

  private enum DisablePattern {
    TRANS_FLAG,
    RAINBOW,
    FIRE_FLY;

    public static DisablePattern getRandom() {
      DisablePattern[] allPatterns = DisablePattern.values();
      return allPatterns[(int) Math.floor(Math.random() * (allPatterns.length))];
    }
  }

  public void clearLEDs() {
    setFullStrip(RGB.BLACK);
  }

  public void displayCriticalError() {
    // gamePieceCandle.setLEDs(255, 0, 0);
  }
}
