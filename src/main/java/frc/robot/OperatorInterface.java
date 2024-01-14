package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Map;

public class OperatorInterface {
  // Operator Interface (OI) class containing all control information

  private static final int kDriverJoystickPort = 0;
  private static final int kOperatorJoystickPort = 1;

  public static final class Driver {
    private static final XboxController driverRumbleController = new XboxController(kDriverJoystickPort);
    private static enum Button {
      A(1),
      B(2),
      X(3),
      Y(4),
      LB(5), // Left Bumper
      RB(6), // Right Bumper
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

      Button(int ID) {
        this.buttonID = ID;
        this.buttonAction = "";
      }

      private int getButtonID() {
        return this.buttonID;
      }
      ;

      private String getButtonAction() {
        return this.buttonAction;
      }

      private void setButtonAction(String name) {
        this.buttonAction = name;
      }
    };
    
    private static final Joystick kJoystick = new Joystick(OperatorInterface.kDriverJoystickPort);

    private static final Button kOrientationButton = Button.Start; // Toggle swerve orientation
    private static final Button kZeroButton = Button.Back; // Zero the gyroscope
    private static final Button kOuttakeButton = Button.RB; // Run outtake
    private static final Button kIntakeButton = Button.LB; // Run intake   //LB is double mapped with Intake and Reset Field Rotation. This should be fixed.
    private static final Button kAlignForwardButton = Button.Y; // Align forwards
    private static final Button kAlignBackwardButton = Button.X; // Align backwards
    private static final Button kBrakeButton = Button.A; // Brake
    private static final Button kPointButton = Button.B; // Point
    private static final Button kResetRotationButton = Button.LB; // Reset Field Rotation

    private static final int kXTranslationAxis = 0;
    private static final int kYTranslationAxis = 1;
    private static final int kRotationAxis = 4;

    // TODO: Tune curves to driver preference
    private static final ControlCurve kXTranslationCurve = new ControlCurve(0.85, 0.05, 0.85, 0.1);
    private static final ControlCurve kYTranslationCurve = new ControlCurve(0.85, 0.05, 0.85, 0.1);
    private static final ControlCurve kRotationCurve = new ControlCurve(0.8, 0, 1, 0.1);

    public static Double getXTranslationSupplier() {
      // This axis is inverted
      return kXTranslationCurve.calculate(-kJoystick.getRawAxis(kXTranslationAxis));
    }

    public static Double getYTranslationSupplier() {
      // This axis is inverted
      return kYTranslationCurve.calculate(-kJoystick.getRawAxis(kYTranslationAxis));
    }

    public static Double getRotationSupplier() {
      // This axis is inverted
      return kRotationCurve.calculate(-kJoystick.getRawAxis(kRotationAxis));
    }

    public static JoystickButton getOrientationButton() {
      kOrientationButton.setButtonAction("Toggle swerve orientation");
      return new JoystickButton(kJoystick, kOrientationButton.getButtonID());
    }

    public static JoystickButton getZeroButton() {
      kZeroButton.setButtonAction("Zero the gyroscope");
      return new JoystickButton(kJoystick, kZeroButton.getButtonID());
    }

    public static JoystickButton getAlignForwardButton() {
      kAlignForwardButton.setButtonAction("Align forward");
      return new JoystickButton(kJoystick, kAlignForwardButton.getButtonID());
    }

    public static JoystickButton getAlignBackButton() {
      kAlignBackwardButton.setButtonAction("Align backward");
      return new JoystickButton(kJoystick, kAlignBackwardButton.getButtonID());
    }

    public static JoystickButton getIntakeButton() {
      kIntakeButton.setButtonAction("Intake");
      return new JoystickButton(kJoystick, kIntakeButton.getButtonID());
    }

    public static JoystickButton getOuttakeButton() {
      kOuttakeButton.setButtonAction("Outtake");
      return new JoystickButton(kJoystick, kOuttakeButton.getButtonID());
    }

    public static JoystickButton getBrakeButton() {
      kOuttakeButton.setButtonAction("Brake");
      return new JoystickButton(kJoystick, kBrakeButton.getButtonID());
    }

    public static JoystickButton getPointButton() {
      kOuttakeButton.setButtonAction("Point");
      return new JoystickButton(kJoystick, kPointButton.getButtonID());
    }

    public static JoystickButton getResetRotationButton() {
      kOuttakeButton.setButtonAction("Reset Rotation");
      return new JoystickButton(kJoystick, kResetRotationButton.getButtonID());
    }
    public static void setRumble(double rumbleIntensity){
      driverRumbleController.setRumble(RumbleType.kBothRumble, rumbleIntensity);
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

      Button(int ID) {
        this.buttonID = ID;
        this.buttonAction = "";
      }

      private int getButtonID() {
        return this.buttonID;
      }
      ;

      private String getButtonAction() {
        return this.buttonAction;
      }

      private void setButtonAction(String name) {
        this.buttonAction = name;
      }
    };

    private static final Joystick kJoystick = new Joystick(OperatorInterface.kOperatorJoystickPort);
    private static final Button kExampleButton =
        Button.LB; // Toggles intake mode between cone and cube

    public static JoystickButton getExampleButton() {
      kExampleButton.setButtonAction("Example");
      return new JoystickButton(kJoystick, kExampleButton.getButtonID());
    }
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
