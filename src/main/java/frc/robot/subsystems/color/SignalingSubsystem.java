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
import frc.robot.subsystems.color.patterns.TransFlag;
import java.util.ArrayList;
import java.util.function.Consumer;

public class SignalingSubsystem extends SubsystemBase {
  private static final int patternUpdateFrequency = 10;

  private final AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;
  // private final CANdle gridPositionCandle;

  private static final int numberOfLEDS = 20;

  private int tick;
  private Timer amplifierTimer;
  private boolean isAllianceAmplified;
  private boolean isOpponentAmplified;

  private final boolean isRedAlliance;
  
  private DisablePattern disablePattern = DisablePattern.getRandom();

  //private final Consumer<Double> driverRumbleConsumer;

  public SignalingSubsystem(int ID, boolean isRedAlliance) {
    //this.driverRumbleConsumer = driverRumbleConsumer;

    tick = 0;
    amplifierTimer = new Timer();
    isAllianceAmplified = false;
    isOpponentAmplified = false;

    this.isRedAlliance = isRedAlliance;

    ledStrip = new AddressableLED(ID);
    ledBuffer = new AddressableLEDBuffer(numberOfLEDS);
    ledStrip.setLength(numberOfLEDS);
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    TransFlag.numberOfLEDS = numberOfLEDS;
    FireFlyPattern.numberOfLEDS = numberOfLEDS;
    BIFlag.numberOfLEDS = numberOfLEDS;
  }

  @Override
  public void periodic() {
    //Alliance Amplification Timer
    if(isAllianceAmplified){
      displayAmplificationTimer(10-(int)amplifierTimer.get(), isRedAlliance?RGB.RED:RGB.BLUE);
      if(amplifierTimer.get()>10){
        isAllianceAmplified = false;
        amplifierTimer.reset();
      }
    }
    //Opponent Amplification Timer
    if(isOpponentAmplified){
      displayAmplificationTimer(10-(int)amplifierTimer.get(), isRedAlliance?RGB.BLUE:RGB.RED);
      if(amplifierTimer.get()>10){
        isOpponentAmplified = false;
        amplifierTimer.reset();
      }
    }
  }

  public void startAmplification(boolean isOpposingTeam){
    amplifierTimer.reset();
    amplifierTimer.start();
    if(isOpposingTeam){
      isOpponentAmplified = true;
    }
    else{
      isAllianceAmplified = true;
    }
  }

  private void setFullStrip(RGB rgb) {
    setSection(rgb, 0, numberOfLEDS);
  }

  private void setSection(RGB rgb, int startID, int count) {
    for(var i = startID; i<Math.min(startID+count, numberOfLEDS); i++){
      ledBuffer.setRGB(i, rgb.red, rgb.green, rgb.blue);
    }
    ledStrip.setData(ledBuffer);
  }

  private void displayAmplificationTimer(int timeRemaining, RGB rgb){
    for(var i = 0; i <= numberOfLEDS/10; i++){
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
      tick++;
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
        //startRainbowAnimation();
        return;
      case TRANS_FLAG:
        pattern = TransFlag.getPattern();
        patternLength = TransFlag.getPatternLength();
        break;
      default:
        //startRainbowAnimation();
        return;
    }
    //stopRainbowAnimation();
    int patternIndex = 0;
    tick %= patternLength;
    int LEDIndex = -tick;
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

  public void displayCriticalError() {
    //gamePieceCandle.setLEDs(255, 0, 0);
  }
}
