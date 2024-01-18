package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import java.util.function.Supplier;

public class OI {
  // Operator Interface (OI) class containing all control information

  private static final int driverJoystickPort = 0;
  private static final int operatorJoystickPort = 1;

  public static final class Driver {
    private static enum Button {
      A(1),
      B(2),
      X(3),
      Y(4),
      LB(5), // Left Bumper
      RB(6), // Right Bumper
      LT(2, 0.5), // Left Trigger
      RT(3, 0.5), // Right Trigger
      Back(7),
      Start(8),
      LJ(9), // Left Joystick Button
      RJ(10), // Right Joystick Button
      POVUP(0),
      POVDOWN(180),
      POVLEFT(270),
      POVRIGHT(90);

      private final int buttonID;
      private String buttonAction;
      private double triggerDetent; // Percentage where axis is triggered as a button

      Button(int ID) {
        this.buttonID = ID;
        this.buttonAction = "";
      }

      Button(int ID, double axisDetent) {
        this(ID);
        this.triggerDetent = axisDetent;
      }

      private int getButtonID() {
        return this.buttonID;
      }

      private double getTriggerDetent() {
        return this.triggerDetent;
      }

      private JoystickButton buildButton(String action) {
        buttonAction = action;
        return new JoystickButton(joystick, getButtonID());
      }

      private Trigger buildTrigger(String action) {
        // "Trigger" referring to the type of button, not the WPI class
        buttonAction = action;
        return new Trigger(() -> joystick.getRawAxis(getButtonID()) > getTriggerDetent());
      }

      private String getButtonAction() {
        return this.buttonAction;
      }
    };

    private static final Joystick joystick = new Joystick(driverJoystickPort);
    private static final XboxController rumbleController = new XboxController(driverJoystickPort);

    private static final Button orientationButton = Button.Start; // Toggle swerve orientation
    private static final Button outtakeButton = Button.RB; // Run outtake
    private static final Button intakeButton = Button.LT; // Run intake
    private static final Button brakeButton = Button.A; // Brake
    private static final Button resetRotationButton = Button.Back; // Reset field rotation

    private static final int xTranslationAxis = 0;
    private static final int yTranslationAxis = 1;
    private static final int rotationAxis = 4;

    // TODO: Tune curves to driver preference
    private static final ControlCurve xTranslationCurve = new ControlCurve(0.85, 0.05, 0.85, 0.1);
    private static final ControlCurve yTranslationCurve = new ControlCurve(0.85, 0.05, 0.85, 0.1);
    private static final ControlCurve rotationCurve = new ControlCurve(0.8, 0, 1, 0.1);

    public static Supplier<Double> getXTranslationSupplier() {
      // This axis is inverted
      return () -> xTranslationCurve.calculate(-joystick.getRawAxis(xTranslationAxis));
    }

    public static Supplier<Double> getYTranslationSupplier() {
      // This axis is inverted
      return () -> yTranslationCurve.calculate(-joystick.getRawAxis(yTranslationAxis));
    }

    public static Supplier<Double> getRotationSupplier() {
      // This axis is inverted
      return () -> rotationCurve.calculate(-joystick.getRawAxis(rotationAxis));
    }

    public static JoystickButton getOrientationButton() {
      return orientationButton.buildButton("Toggle swerve orientation");
    }

    public static Trigger getIntakeButton() {
      return intakeButton.buildTrigger("Intake");
    }

    public static JoystickButton getOuttakeButton() {
      return outtakeButton.buildButton("Outtake");
    }

    public static JoystickButton getBrakeButton() {
      return brakeButton.buildButton("Brake");
    }

    public static JoystickButton getResetRotationButton() {
      return resetRotationButton.buildButton("Reset Rotation");
    }

    public static void setRumble(double rumbleIntensity) {
      rumbleController.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }
  }

  public static final class Operator {
    private static enum Button {
      A(1),
      B(2),
      X(3),
      Y(4),
      LB(5), // Left Bumper
      RB(6), // Right Bumper
      LT(2, 0.5), // Left Trigger
      RT(3, 0.5), // Right Trigger
      Back(7),
      Start(8),
      LJ(9), // Left Joystick Button
      RJ(10), // Right Joystick Button
      POVUP(0),
      POVDOWN(180),
      POVLEFT(270),
      POVRIGHT(90);

      private final int buttonID;
      private String buttonAction;
      private double triggerDetent; // Percentage where axis is triggered as a button

      Button(int ID) {
        this.buttonID = ID;
        this.buttonAction = "";
      }

      Button(int ID, double axisDetent) {
        this(ID);
        this.triggerDetent = axisDetent;
      }

      private int getButtonID() {
        return this.buttonID;
      }

      private double getTriggerDetent() {
        return this.triggerDetent;
      }

      private JoystickButton buildButton(String action) {
        buttonAction = action;
        return new JoystickButton(joystick, getButtonID());
      }

      private Trigger buildTrigger(String action) {
        // "Trigger" referring to the type of button, not the WPI class
        buttonAction = action;
        return new Trigger(() -> joystick.getRawAxis(getButtonID()) > getTriggerDetent());
      }

      private String getButtonAction() {
        return this.buttonAction;
      }
    };

    private static final Joystick joystick = new Joystick(operatorJoystickPort);
  }

  public static void putControllerButtons() {
    ShuffleboardLayout driverButtonsLayout =
        Shuffleboard.getTab("Controls")
            .getLayout("Driver Buttons", BuiltInLayouts.kList)
            .withSize(2, 5)
            .withPosition(0, 0)
            .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for Variables;

    ShuffleboardLayout operatorButtonsLayout =
        Shuffleboard.getTab("Controls")
            .getLayout("Operator Buttons", BuiltInLayouts.kList)
            .withSize(2, 5)
            .withPosition(2, 0)
            .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for Variables;

    for (Driver.Button button : Driver.Button.values()) {
      driverButtonsLayout.add(
          String.valueOf(button.getButtonID()),
          "Button " + button.toString() + ": " + button.getButtonAction());
    }

    for (Operator.Button button : Operator.Button.values()) {
      operatorButtonsLayout.add(
          String.valueOf(button.getButtonID() + Operator.Button.values().length),
          "Button " + button.toString() + ": " + button.getButtonAction());
    }
  }

  public static class ControlCurve {
    private double ySaturation; // Maximum output, in percentage of possible output
    private double yIntercept; // Minimum output, in percentage of saturation
    private double curvature; // Curvature shift between linear and cubic
    private double deadzone; // Range of input that will always return zero output

    public ControlCurve(double ySaturation, double yIntercept, double curvature, double deadzone) {
      this.ySaturation = ySaturation;
      this.yIntercept = yIntercept;
      this.curvature = curvature;
      this.deadzone = deadzone;
    }

    public double calculate(double input) {
      /* https://www.desmos.com/calculator/w6ovblmmqj
      Two equations, separated by a ternary
      The first is the deadzone
      y = 0 {|x| < d}
      The second is the curve
      y = a(sign(x) * b + (1 - b) * (c * x^3 + (1 - c) * x)) {|x| >= d}
      Where
      x = input
      y = output
      a = ySaturation
      b = yIntercept
      c = curvature
      d = deadzone
      and 0 <= a,b,c,d < 1
      */
      return Math.abs(input) < deadzone
          ? 0
          : ySaturation
              * (Math.signum(input) * yIntercept
                  + (1 - yIntercept) * (curvature * Math.pow(input, 3) + (1 - curvature) * input));
    }
  }
}
