package frc.robot.subsystems.signaling;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.PlacementMode;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.signaling.patterns.FireFlyPattern;
import frc.robot.subsystems.signaling.patterns.PatternNode;
import frc.robot.subsystems.signaling.patterns.RainbowPattern;
import frc.robot.subsystems.signaling.patterns.TransFlag;
import java.util.function.Consumer;

public class SignalingSubsystem extends SubsystemBase {

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  private int tick;
  private int patternTick;
  private final Timer rumbleTimer;
  private double rumbleEndTime = 10;

  private DisablePattern disablePattern = DisablePattern.getRandom();

  private final Consumer<Double> driverRumbleConsumer;

  private final RobotStateManager robotStateManager;
  private final Trigger isAmpModeTrigger;

  public SignalingSubsystem(
      final Consumer<Double> driverRumbleConsumer, final RobotStateManager robotStateManager) {
    this.driverRumbleConsumer = driverRumbleConsumer;
    this.robotStateManager = robotStateManager;
    this.isAmpModeTrigger = robotStateManager.isAmpTrigger();

    tick = 0;
    patternTick = 0;
    rumbleTimer = new Timer();

    // Initialize LED Strip
    ledStrip = new AddressableLED(Constants.LED_PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(Constants.NUMBER_OF_LEDS);
    ledStrip.setLength(Constants.NUMBER_OF_LEDS);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  @Override
  public void periodic() {
    // Update Light Pattern
    if (DriverStation.isDisabled()) updatePattern();
    else {
      isAmpModeTrigger.onFalse(runOnce(() -> resetLEDs()));
      isAmpModeTrigger.onTrue(runOnce(() -> resetLEDs()));
    }

    // End Signaling
    if (rumbleTimer.get() > rumbleEndTime) {
      endSignal();
    }
  }

  public void resetLEDs() {
    setHalfStrip(robotStateManager.getPlacementMode() == PlacementMode.AMP ? RGB.BLUE : RGB.RED);
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

  public void startIntakeSignal() {
    startSignal(10, Constants.OperatorConstants.RUMBLE_STRENGTH, RGB.WHITE);
  }

  public void startAmpSignal() {
    startSignal(10, Constants.OperatorConstants.RUMBLE_STRENGTH, RGB.RED);
  }

  public void startShooterSignal() {
    startSignal(10, Constants.OperatorConstants.RUMBLE_STRENGTH, RGB.BLUE);
  }

  public void endSignal() {
    rumbleTimer.reset();
    rumbleTimer.stop();
    driverRumbleConsumer.accept(0.0);
    resetLEDs();
  }

  private void setFullStrip(final RGB rgb) {
    setSection(rgb, 0, Constants.NUMBER_OF_LEDS);
  }

  private void setHalfStrip(final RGB rgb) {
    setFullStrip(RGB.BLACK);
    for (var i = 0; i < Constants.NUMBER_OF_LEDS; i += 2) {
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
      if (i > -1 && i < Constants.NUMBER_OF_LEDS) {
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
    if (tick > Constants.PATTERN_SPEED * 50) {
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
    while (LEDIndex < Constants.NUMBER_OF_LEDS) {
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
    ledStrip.setData(ledBuffer);
  }
}
