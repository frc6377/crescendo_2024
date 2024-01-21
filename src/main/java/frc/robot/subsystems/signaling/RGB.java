package frc.robot.subsystems.signaling;

import java.util.function.IntSupplier;

public class RGB {
  private static final int MAX_RGB_VALUE = 255;
  public static final RGB BLACK = new RGB(0, 0, 0);
  public static final RGB WHITE = new RGB(MAX_RGB_VALUE, MAX_RGB_VALUE, MAX_RGB_VALUE);

  public static final RGB RED = new RGB(MAX_RGB_VALUE, 0, 0);
  public static final RGB ORANGE = new RGB(255, 130, 0);
  public static final RGB YELLOW = new RGB(MAX_RGB_VALUE, MAX_RGB_VALUE, 0);
  public static final RGB GREEN = new RGB(0, MAX_RGB_VALUE, 0);
  public static final RGB BLUE = new RGB(0, 0, MAX_RGB_VALUE);
  public static final RGB HOWDY_BLUE = new RGB(0, 225, 225);
  public static final RGB PURPLE = new RGB(127, 0, 255);
  public static final RGB PINK = new RGB(174, 47, 132);
  public static final RGB[] RainbowThing =
      new RGB[] {RED, ORANGE, YELLOW, GREEN, HOWDY_BLUE, PURPLE};

  public static final RGB HOWDY_TAN = new RGB(229, 218, 91);
  public static final RGB HOWDY_BROWN_1 = new RGB(140, 101, 64);
  public static final RGB HOWDY_BROWN_2 = new RGB(97, 71, 5);
  public static final RGB HOWDY_BROWN_3 = new RGB(148, 88, 5);

  public static RGB randomColor() {
    final IntSupplier rndValue = () -> (int) Math.round(Math.random() * (MAX_RGB_VALUE + 1));

    return new RGB(rndValue.getAsInt(), rndValue.getAsInt(), rndValue.getAsInt());
  }

  public RGB(final int red, final int green, final int blue) {
    this(red, green, blue, 0);
  }

  public RGB(final int red, final int green, final int blue, final int white) {
    this.red = red;
    this.green = green;
    this.blue = blue;
    this.white = white;
  }

  public final int red;
  public final int green;
  public final int blue;
  public final int white;

  @Override
  public boolean equals(final Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    final RGB that = (RGB) o;
    return red == that.red && green == that.green && blue == that.blue && white == that.white;
  }
}
