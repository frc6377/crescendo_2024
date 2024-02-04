package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import java.util.function.Supplier;

public class OI {
  // Operator Interface (OI) class containing all control information

  private static final int driverJoystickPort = 0;
  private static final int operatorJoystickPort = 1;

  public static final class Driver {
    private static final XboxController controller = new XboxController(driverJoystickPort);

    // A B Y X Buttons
    public static final Control speakerRotateButton =
        new Control(XboxController.Button.kA, "Rotate to speaker", controller);
    public static final Control trapScoreButton =
        new Control(XboxController.Button.kB, "Score Trap", controller);
    public static final Control sourceIntakeButton =
        new Control(XboxController.Button.kX, "Intake Source", controller);
    public static final Control groundIntakeButton =
        new Control(XboxController.Button.kY, "Intake Ground", controller);

    // Bumpers & Triggers
    public static final Control intakeTrigger =
        new Control(XboxController.Axis.kLeftTrigger, "Run intake", controller, 0.5);
    public static final Control ampScoreTrigger =
        new Control(XboxController.Axis.kRightTrigger, "Score amp", controller, 0.5);
    public static final Control outtakeButton =
        new Control(XboxController.Button.kLeftBumper, "Run outtake", controller);
    public static final Control highGearButton =
        new Control(XboxController.Button.kRightBumper, "High gear", controller);

    // Start, End & Left/Right stick buttons
    public static final Control resetRotationButton =
        new Control(XboxController.Button.kStart, "Reset field rotation", controller);
    public static final Control pointDriveLeftButton =
        new Control(XboxController.Button.kLeftStick, "Point drive", controller);
    public static final Control pointDriveRightButton =
        new Control(XboxController.Button.kRightStick, "Point drive", controller);
    public static final Control orientationButton =
        new Control(XboxController.Button.kBack, "Toggle swerve orientation", controller);

    // Directional Pad
    public static final Control sourceGuidanceButton =
        new Control(0, "Guide to Source", controller);
    public static final Control ampGuidanceLeftButton =
        new Control(270, "Guide to Amp", controller);
    public static final Control ampGuidanceRightButton =
        new Control(90, "Guide to Amp", controller);

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.1);
    private static final ControlCurve rotationCurve = new ControlCurve(0.8, 0, 1, 0.1, true);

    // Joystick Axes
    public static final Control xTranslationAxis =
        new Control(XboxController.Axis.kLeftX, "X Translation", controller, xTranslationCurve);
    public static final Control yTranslationAxis =
        new Control(XboxController.Axis.kLeftY, "Y Translation", controller, yTranslationCurve);
    public static final Control rotationAxis =
        new Control(XboxController.Axis.kRightX, "Rotation", controller, rotationCurve);
    public static final Control RightY =
        new Control(XboxController.Axis.kRightY, null, controller, null);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }
  }

  public static final class Operator {
    private static final XboxController controller = new XboxController(operatorJoystickPort);

    // A B Y X Buttons
    public static final Control A = new Control(XboxController.Button.kA, null, controller);
    public static final Control zeroArmButton =
        new Control(XboxController.Button.kB, "Zero arm", controller);
    public static final Control Y = new Control(XboxController.Button.kY, null, controller);
    public static final Control X = new Control(XboxController.Button.kX, null, controller);

    // Bumpers & Triggers
    public static final Control LT =
        new Control(XboxController.Axis.kLeftTrigger, null, controller, 0.5);
    public static final Control RT =
        new Control(XboxController.Axis.kRightTrigger, null, controller, 0.5);
    public static final Control LB =
        new Control(XboxController.Button.kLeftBumper, null, controller);
    public static final Control RB =
        new Control(XboxController.Button.kRightBumper, null, controller);

    // Start, End & Left/Right stick buttons
    public static final Control start = new Control(XboxController.Button.kStart, null, controller);
    public static final Control back = new Control(XboxController.Button.kBack, null, controller);
    public static final Control LSB =
        new Control(XboxController.Button.kLeftStick, null, controller);
    public static final Control RSB =
        new Control(XboxController.Button.kRightStick, null, controller);

    // Directional Pad
    public static final Control toggleSourceAutopilotButton =
        new Control(0, "Toggle source autopilot", controller);
    public static final Control toggleAmpAutopilotLeftButton =
        new Control(270, "Toggle amp autopilot", controller);
    public static final Control toggleAmpAutopilotRightButton =
        new Control(90, "Toggle amp autopilot", controller);

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 0, 0.0, true);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.1);
    private static final ControlCurve rotationCurve = new ControlCurve(0.8, 0, 1, 0.1, true);

    // Joystick Axes
    public static final Control xTranslationAxis =
        new Control(XboxController.Axis.kLeftX, "X Translation", controller, xTranslationCurve);
    public static final Control yTranslationAxis =
        new Control(XboxController.Axis.kLeftY, "Y Translation", controller, yTranslationCurve);
    public static final Control rotationAxis =
        new Control(XboxController.Axis.kRightX, "Rotation", controller, rotationCurve);
    public static final Control RightY =
        new Control(XboxController.Axis.kRightY, null, controller, null);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }
  }

  // --- OI UTILITIES -- //

  public static Supplier<Double> getAxisSupplier(Control axis) {
    if (axis.getType() != Control.ControlType.AXIS) {
      DriverStation.reportError(axis.getAction() + " is not an axis", true);
      return () -> 0d;
    }
    return () -> axis.getCurve().calculate(axis.getController().getRawAxis(axis.getId()));
  }

  public static JoystickButton getButton(Control button) {
    if (button.getType() != Control.ControlType.BUTTON) {
      DriverStation.reportError(button.getAction() + " is not a button", true);
    }
    return new JoystickButton(button.getController(), button.getId());
  }

  public static POVButton getPOVButton(Control povButton) {
    if (povButton.getType() != Control.ControlType.POVBUTTON) {
      DriverStation.reportError(povButton.getAction() + " is not a POV button", true);
    }
    return new POVButton(povButton.getController(), povButton.getId());
  }

  public static Trigger getTrigger(Control trigger) {
    // "Trigger" referring to the type of button, not the WPI class
    if (trigger.getType() != Control.ControlType.TRIGGER) {
      DriverStation.reportError(trigger.getAction() + " is not a trigger", true);
      return new Trigger(() -> false);
    }
    return new Trigger(
        () -> trigger.getController().getRawAxis(trigger.getId()) > trigger.getThreshold());
  }

  private static class Control {
    private enum ControlType {
      AXIS,
      BUTTON,
      POVBUTTON,
      TRIGGER
    }

    private int id;
    private String action;
    private String name; // Refers to button name
    private XboxController controller;
    private ControlCurve curve;
    private double threshold; // Percentage where axis is triggered as a button
    private ControlType type;

    private Control(
        int id, String action, String name, XboxController controller, ControlType type) {
      this.id = id;
      this.action = action;
      this.name = name;
      this.controller = controller;
      this.type = type;
      putControl();
    }

    Control(
        XboxController.Axis axis, String action, XboxController controller, ControlCurve curve) {
      this(axis.value, action, axis.name(), controller, ControlType.AXIS);
      this.curve = curve;
    }

    Control(XboxController.Axis axis, String action, XboxController controller) {
      this(axis, action, controller, new ControlCurve(1, 0, 0, 0));
    }

    Control(XboxController.Button button, String action, XboxController controller) {
      this(button.value, action, button.name(), controller, ControlType.BUTTON);
    }

    Control(double povAngle, String action, XboxController controller) {
      this((int) povAngle, action, "POV " + povAngle, controller, ControlType.POVBUTTON);
    }

    Control(XboxController.Axis axis, String action, XboxController controller, double threshold) {
      this(axis.value, action, axis.name(), controller, ControlType.TRIGGER);
      this.threshold = threshold;
    }

    private int getId() {
      return id;
    }

    private String getAction() {
      return action;
    }

    private String getName() {
      return name;
    }

    private XboxController getController() {
      return controller;
    }

    private ControlCurve getCurve() {
      return curve;
    }

    private double getThreshold() {
      return threshold;
    }

    private ControlType getType() {
      return type;
    }

    private void putControl() {
      if (getAction() != null) {
        if (controller.getPort() == driverJoystickPort) {
          driverControlsLayout.add(
              "Driver " + getType().toString() + " " + String.valueOf(getId()),
              type.toString() + " " + getName() + ": " + getAction());
        }
        if (controller.getPort() == operatorJoystickPort) {
          operatorControlsLayout.add(
              "Operator " + getType().toString() + " " + String.valueOf(getId()),
              type.toString() + " " + getName() + ": " + getAction());
        }
      }
    }
  }

  public static class ControlCurve {
    private final double ySaturation; // Maximum output, in percentage of possible output
    private final double yIntercept; // Minimum output, in percentage of saturation
    private final double curvature; // Curvature shift between linear and cubic
    private final double deadzone; // Range of input that will always return zero output
    private final boolean inverted;

    public ControlCurve(
        double ySaturation,
        double yIntercept,
        double curvature,
        double deadzone,
        boolean inverted) {
      this.ySaturation = ySaturation;
      this.yIntercept = yIntercept;
      this.curvature = curvature;
      this.deadzone = deadzone;
      this.inverted = inverted;
    }

    public ControlCurve(double ySaturation, double yIntercept, double curvature, double deadzone) {
      this(ySaturation, yIntercept, curvature, deadzone, false);
    }

    public double calculate(double input) {
      /* https://www.desmos.com/calculator/w6ovblmmqj
      First is the deadzone
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
      if (Math.abs(input) < deadzone) {
        return 0;
      }
      return (inverted ? -1 : 1)
          * ySaturation
          * (Math.signum(input) * yIntercept
              + (1 - yIntercept) * (curvature * Math.pow(input, 3) + (1 - curvature) * input));
    }
  }

  private static ShuffleboardLayout driverControlsLayout =
      Shuffleboard.getTab("Controls")
          .getLayout("Driver Controls", BuiltInLayouts.kList)
          .withSize(3, 5)
          .withPosition(0, 0)
          .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for Variables;

  private static ShuffleboardLayout operatorControlsLayout =
      Shuffleboard.getTab("Controls")
          .getLayout("Operator Controls", BuiltInLayouts.kList)
          .withSize(3, 5)
          .withPosition(3, 0)
          .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for Variables;
}
