package frc.robot.subsystems.color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.color.patterns.BIFlag;
import frc.robot.subsystems.color.patterns.FireFlyPattern;
import frc.robot.subsystems.color.patterns.PatternNode;
import frc.robot.subsystems.color.patterns.RainbowPattern;
import frc.robot.subsystems.color.patterns.TransFlag;
import java.util.ArrayList;
import java.util.function.Consumer;

public class SignalingSubsystem extends SubsystemBase {
  private static final int patternUpdateFrequency = 10;

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  
  private static final int numberOfLEDS = Constants.LED_COUNT;

  private int tick;
  private int patternTick;
  private final Timer amplifierTimer;
  private final Timer rumbleTimer;
  private double rumbleEndTime = 0;
  private boolean isAllianceAmplified;
  private boolean isOpponentAmplified;

  private final boolean isRedAlliance;

  private DisablePattern disablePattern = DisablePattern.getRandom();

  private final Consumer<Double> driverRumbleConsumer;

  public SignalingSubsystem(final int ID, final Consumer<Double> driverRumbleConsumer) {
    this.driverRumbleConsumer = driverRumbleConsumer;

    tick = 0;
    patternTick = 0;
    amplifierTimer = new Timer();
    rumbleTimer = new Timer();
    isAllianceAmplified = false;
    isOpponentAmplified = false;

    this.isRedAlliance = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);

    // Initialize LED Strip
    ledStrip = new AddressableLED(ID);
    ledBuffer = new AddressableLEDBuffer(numberOfLEDS);
    ledStrip.setLength(numberOfLEDS);
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    // Setup Disable Patterns
    TransFlag.numberOfLEDS = numberOfLEDS;
    FireFlyPattern.numberOfLEDS = numberOfLEDS;
    BIFlag.numberOfLEDS = numberOfLEDS;
    RainbowPattern.numberOfLEDS = numberOfLEDS;
  }

  @Override
  public void periodic() {
    // Update Light Pattern
    if (DriverStation.isDisabled()) updatePattern();

    // End Signaling
    if (rumbleTimer.get() > rumbleEndTime) {
      rumbleTimer.reset();
      driverRumbleConsumer.accept(0.0);
      setFullStrip(RGB.BLACK);
    }
    // Alliance Amplification Timer
    if (isAllianceAmplified) {
      displayAmplificationTimer(
          10 - (int) amplifierTimer.get(), isRedAlliance ? RGB.RED : RGB.BLUE);
      if (amplifierTimer.get() > 10) {
        isAllianceAmplified = false;
        amplifierTimer.reset();
        startSignal(Constants.AMPLIFICATION_RUMBLE_TIME, Constants.AMPLIFICATION_RUMBLE_INTENSITY);
      }
    }
    // Opponent Amplification Timer
    else if (isOpponentAmplified) {
      displayAmplificationTimer(
          10 - (int) amplifierTimer.get(), isRedAlliance ? RGB.BLUE : RGB.RED);
      if (amplifierTimer.get() > 10) {
        isOpponentAmplified = false;
        amplifierTimer.reset();
        startSignal(Constants.AMPLIFICATION_RUMBLE_TIME, Constants.AMPLIFICATION_RUMBLE_INTENSITY);
      }
    }
  }

  public void startAmplification(final boolean isOpposingTeam) {
    amplifierTimer.reset();
    amplifierTimer.start();
    startSignal(Constants.AMPLIFICATION_RUMBLE_TIME, Constants.AMPLIFICATION_RUMBLE_INTENSITY);
    if (isOpposingTeam) {
      isOpponentAmplified = true;
    } else {
      isAllianceAmplified = true;
    }
  }

  public void endAmplification(final boolean isOpposingTeam) {
    if (isOpposingTeam) {
      isOpponentAmplified = false;
    } else {
      isAllianceAmplified = false;
    }
    amplifierTimer.reset();
    startSignal(Constants.AMPLIFICATION_RUMBLE_TIME, Constants.AMPLIFICATION_RUMBLE_INTENSITY);
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
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  private void startSignal(final double time, final double intensity, final RGB rgb) {
    driverRumbleConsumer.accept(intensity);
    rumbleEndTime = time;
    setFullStrip(rgb);
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  private void setFullStrip(final RGB rgb) {
    setSection(rgb, 0, numberOfLEDS);
  }

  private void setSection(final RGB rgb, final int startID, final int count) {
    for (var i = Math.max(startID, 0); i < Math.min(startID + count, numberOfLEDS); i++) {
      ledBuffer.setRGB(i, rgb.red, rgb.green, rgb.blue);
    }
    ledStrip.setData(ledBuffer);
  }

  private void displayAmplificationTimer(final int timeRemaining, final RGB rgb) {
    for (var i = 0; i <= numberOfLEDS / 10; i++) {
      setSection(rgb, i * 10, timeRemaining);
      setSection(RGB.BLACK, i * 10 + timeRemaining, 10 - timeRemaining);
    }
    ledStrip.setData(ledBuffer);
  }

  private void updatePattern() {
    PatternNode[] pattern;
    int patternLength;

    tick++;
    if (tick > patternUpdateFrequency) {
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
    int LEDIndex = -patternTick;
    while (LEDIndex < numberOfLEDS) {
      patternIndex %= pattern.length;

      PatternNode node = pattern[patternIndex];
      RGB c = pattern[patternIndex].color;

      setSection(c, LEDIndex, node.repeat);
      LEDIndex += node.repeat;
      patternIndex += 1;
    }
  }

  public void randomizePattern() {
    disablePattern = DisablePattern.getRandom();
  }

  private enum DisablePattern {
    TRANS_FLAG,
    RAINBOW,
    FIRE_FLY;

    public static DisablePattern getRandom() {
      // Do not use due to special request
      DisablePattern[] DNU = {DisablePattern.TRANS_FLAG};
      DisablePattern[] allPatterns = DisablePattern.values();
      ArrayList<DisablePattern> useable = new ArrayList<>();
      for (DisablePattern p : allPatterns) {
        boolean skip = false;
        for (DisablePattern d : DNU) {
          if (p == d) skip = true;
        }
        if (skip) continue;
        useable.add(p);
      }

      return useable.get((int) Math.floor(Math.random() * (useable.size())));
    }
  }

  public void clearLEDs() {
    setFullStrip(RGB.BLACK);
  }

  public void displayCriticalError() {
    // gamePieceCandle.setLEDs(255, 0, 0);
  }
}
