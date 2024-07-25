package frc.robot.devMode;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.OI.ControlCurve;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DevOI {
  private static final int driverJoystickPort = 0;
  private static final int operatorJoystickPort = 1;

  public static final class Driver {
    public static final XboxController controller = new XboxController(driverJoystickPort);

    // A B Y X Buttons
    public static final TriggerControl retractClimber = getControl(XboxController.Button.kA, null);
    public static final TriggerControl latchClimber = getControl(XboxController.Button.kB, null);
    public static final TriggerControl simple = getControl(XboxController.Button.kX, null);
    public static final TriggerControl prepClimb = getControl(XboxController.Button.kY, null);

    // Bumpers & Triggers
    public static final TriggerControl leftTrigger =
        getControl(XboxController.Axis.kLeftTrigger, null, 0.5);
    public static final TriggerControl rightTrigger =
        getControl(XboxController.Axis.kRightTrigger, null, 0.5);
    public static final TriggerControl leftBumper =
        getControl(XboxController.Button.kLeftBumper, null);
    public static final TriggerControl highGear =
        getControl(XboxController.Button.kRightBumper, "High Gear");

    // Start, End & Left/Right stick buttons
    public static final TriggerControl start = getControl(XboxController.Button.kStart, null);
    public static final TriggerControl leftStick =
        getControl(XboxController.Button.kLeftStick, null);
    public static final TriggerControl rightStick =
        getControl(XboxController.Button.kRightStick, null);
    public static final TriggerControl back = getControl(XboxController.Button.kBack, null);

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.0);
    public static final ControlCurve rotationCurve = new ControlCurve(0.8, 0, 1, 0.0, true);

    // Joystick Axes
    public static final AxisControl xTranslationAxis =
        new AxisControl(XboxController.Axis.kLeftX, null, controller, xTranslationCurve);
    public static final AxisControl yTranslationAxis =
        new AxisControl(XboxController.Axis.kLeftY, null, controller, yTranslationCurve);
    public static final AxisControl rotationAxis =
        new AxisControl(XboxController.Axis.kRightX, null, controller, rotationCurve);
    public static final AxisControl RightY =
        new AxisControl(XboxController.Axis.kRightY, null, controller, null);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }

    private static TriggerControl getControl(
        XboxController.Axis axis, String name, double threshold) {
      return new TriggerControl(axis, name, Driver.controller, threshold);
    }

    private static TriggerControl getControl(XboxController.Button button, String name) {
      return new TriggerControl(button, name, Driver.controller);
    }
  }

  public static final class Operator {
    public static final XboxController controller = new XboxController(operatorJoystickPort);

    // A B Y X Buttons
    public static final TriggerControl aButton = getControl(XboxController.Button.kA, null);
    public static final TriggerControl bButton = getControl(XboxController.Button.kB, null);
    public static final TriggerControl xButton = getControl(XboxController.Button.kX, null);
    public static final TriggerControl yButton = getControl(XboxController.Button.kY, null);

    // Bumpers & Triggers
    public static final TriggerControl fire =
        getControl(XboxController.Axis.kLeftTrigger, "Fire", 0.5);
    public static final TriggerControl prepareToFire =
        getControl(XboxController.Axis.kRightTrigger, "Rev/Prep to score", 0.5);
    public static final TriggerControl switchToSpeaker =
        getControl(XboxController.Button.kLeftBumper, "Speaker Mode");
    public static final TriggerControl switchToAmp =
        getControl(XboxController.Button.kRightBumper, "Amp Mode");

    // Start, End & Left/Right stick buttons
    public static final TriggerControl start = getControl(XboxController.Button.kStart, null);
    public static final TriggerControl leftStick =
        getControl(XboxController.Button.kLeftStick, null);
    public static final TriggerControl rightStick =
        getControl(XboxController.Button.kRightStick, null);
    public static final TriggerControl back = getControl(XboxController.Button.kBack, null);

    // POV Buttons
    public static TriggerControl incrementTrial =
        new TriggerControl(() -> Operator.controller.getPOV() == 0, "Increment Trial");
    public static TriggerControl povRight =
        new TriggerControl(() -> Operator.controller.getPOV() == 90, null);
    public static TriggerControl decrementTrial =
        new TriggerControl(() -> Operator.controller.getPOV() == 180, "Decrement Trial");
    public static TriggerControl povLeft =
        new TriggerControl(() -> Operator.controller.getPOV() == 270, null);

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.0);
    public static final ControlCurve rotationCurve = new ControlCurve(0.8, 0, 1, 0.0, true);

    // Joystick Axes
    public static final AxisControl leftX =
        new AxisControl(XboxController.Axis.kLeftX, null, controller, xTranslationCurve);
    public static final AxisControl leftY =
        new AxisControl(XboxController.Axis.kLeftY, null, controller, yTranslationCurve);
    public static final AxisControl rightX =
        new AxisControl(XboxController.Axis.kRightX, null, controller, rotationCurve);
    public static final AxisControl rightY =
        new AxisControl(XboxController.Axis.kRightY, null, controller, null);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }

    private static TriggerControl getControl(
        XboxController.Axis axis, String name, double threshold) {
      return new TriggerControl(axis, name, Operator.controller, threshold);
    }

    private static TriggerControl getControl(XboxController.Button button, String name) {
      return new TriggerControl(button, name, Operator.controller);
    }
  }

  public static class AxisControl implements Supplier<Double> {
    public String name;
    public int id;
    protected XboxController controller;
    protected ControlCurve controlCurve;

    public AxisControl(
        Axis axis, String name, XboxController controller, ControlCurve controlCurve) {
      id = axis.value;
      this.controller = controller;
      this.controlCurve = controlCurve;
    }

    @Override
    public Double get() {
      return controlCurve.calculate(controller.getRawAxis(id));
    }
  }

  public static class TriggerControl extends Trigger {
    public int id;
    public String name;

    public TriggerControl(BooleanSupplier event, String name) {
      super(event);
      this.name = name;
    }

    public TriggerControl(XboxController.Button button, String name, XboxController controller) {
      this(button.value, name, controller);
    }

    public TriggerControl(int id, String name, XboxController controller) {
      super(() -> controller.getRawButton(id));
      this.id = id;
      this.name = name;
    }

    public TriggerControl(
        XboxController.Axis button, String name, XboxController controller, double threshold) {
      super(() -> controller.getRawAxis(button.value) > threshold);
      this.name = name;
    }
  }
}
