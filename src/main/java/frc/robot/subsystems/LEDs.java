package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.LED;
import frc.robot.Constants.Constants.PWM;
import frc.robot.Constants.Constants.TEAM_COLOR;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class LEDs extends SubsystemBase {
  private static AddressableLED strip;
  private static AddressableLEDBuffer buffer;
  private RobotStateController stateController;
  private static int length = 30;
  private static State state = State.OFF;
  private static double time = 0;
  private static double centerFillTimer = 0;
  private static boolean centerFillCalled = false;

  public static enum State {
    OFF,
    DISABLED,
    ENABLED,
    HAS_VISION_TARGETS,
    DRIVING_AUTO,
    DRIVING_TELEOP_RED,
    DRIVING_TELEOP_BLUE,
    HAS_ALGAE,
    HAS_CORAL,
    CAN_SEE_ALGAE,
    AIMING_PROCESSOR,
    SCORING_PROCESSOR,
    AIMING_BARGE,
    SCORING_BARGE,
    AIMING_REEF,
    SCORING_L2,
    SCORING_L3,
    SCORING_L4,
    HANG
  }

  public static enum Direction {
    LEFT,
    RIGHT,
  }

  public static final int[] WHITE = {255, 255, 255};
  public static final int[] ANTARES_BLUE = {37, 46, 69};
  public static final int[] ANTARES_YELLOW = {242, 222, 139};
  public static final int[] RED = {255, 0, 0};
  public static final int[] GREEN = {0, 255, 0};
  public static final int[] BLUE = {0, 20, 255};
  public static final int[] RSL_ORANGE = {255, 100, 0};
  public static final int[] LIGHT_BLUE = {173, 216, 230};
  public static final int[] YELLOW = {255, 255, 0};
  public static final int[] CYAN = {0, 255, 255};
  public static final int[] DARK_GREEN = {0, 100, 0};
  public static final int[] PURPLE = {108, 59, 170};
  public static final int[] MAGENTA = {255, 0, 255};

  private DoubleSupplier animationSpeed;

  public LEDs(RobotStateController stateController, DoubleSupplier animationSpeed) {
    this.stateController = stateController;
    this.animationSpeed = animationSpeed;

    strip = new AddressableLED(PWM.LEDS);
    buffer = new AddressableLEDBuffer(length);
    strip.setLength(buffer.getLength());

    strip.setData(buffer);
    strip.start();
  }

  @Override
  public void periodic() {
    switch (state) {
      case OFF:
        setColor(0, length, new int[] {0, 0, 0});
        break;
      case DISABLED:
        setColorWave(0, length, WHITE, LIGHT_BLUE, 1.0, Direction.LEFT);
        break;
      case ENABLED:
        setColorWave(0, length, getBumperLEDColor(), new int[] {80, 80, 80}, 1.0, Direction.LEFT);
        break;
      case HAS_VISION_TARGETS:
        setRainbow(0, length);
      case DRIVING_AUTO:
        setColorWave(0, length, ANTARES_YELLOW, ANTARES_BLUE, 1.0, Direction.LEFT);
        break;
      case DRIVING_TELEOP_BLUE:
        setColorWave(0, length, ANTARES_BLUE, 1.0, Direction.LEFT);
        break;
      case DRIVING_TELEOP_RED:
        setColorWave(0, length, WHITE, LIGHT_BLUE, 1.0, Direction.LEFT);
        break;
      case HAS_ALGAE:
        setColorWave(0, length, getBumperLEDColor(), CYAN, 1.0, Direction.LEFT);
        break;
      case HAS_CORAL:
        setColorWave(0, length, getBumperLEDColor(), PURPLE, 1.0, Direction.LEFT);
        break;
      case CAN_SEE_ALGAE:
        setColorFromCenter(0, length, CYAN, 0.25);
        break;
      case AIMING_PROCESSOR:
        setColorFlash(0, length, DARK_GREEN, 5);
        break;
      case SCORING_PROCESSOR:
        setColor(length, DARK_GREEN);
        break;
      case AIMING_BARGE:
        setColorFlash(0, length, YELLOW, 5);
        break;
      case SCORING_BARGE:
        setColor(length, YELLOW);
        break;
      case AIMING_REEF:
        setColorFlash(0, length, MAGENTA, 5);
        break;
      case SCORING_L2:
        setColor(length, MAGENTA);
        break;
      case SCORING_L3:
        setColor(length, MAGENTA);
        break;
      case SCORING_L4:
        setColor(length, MAGENTA);
        break;
      case HANG:
        setColor(length, RSL_ORANGE);
        break;
    }

    // setColorWave(0, length, PURPLE, ANTARES_BLUE, 1.0, Direction.LEFT);

    // setRainbow(0, length);

    strip.setData(buffer);
    clear();

    state = State.OFF;

    time += Robot.getLoopTime() * animationSpeed.getAsDouble();

    if (!centerFillCalled) {
      centerFillTimer = 0;
    } else {
      centerFillTimer += Robot.getLoopTime();
    }
    centerFillCalled = false;
  }

  @Override
  public void simulationPeriodic() {
    // setState(State.DISABLED);
  }

  public static Command setStateCommand(State state) {
    return Commands.run(() -> setState(state));
  }

  public static void setState(State state) {
    if (state.ordinal() > LEDs.state.ordinal()) LEDs.state = state;
  }

  private static void setColor(int pixel, int[] RGB) {
    buffer.setRGB(pixel, RGB[0], RGB[1], RGB[2]);
  }

  private static void setColor(int start, int stop, int[] RGB) {
    for (int pixel = start; pixel < stop; pixel++) {
      setColor(pixel, RGB);
    }
  }

  private static void setTopStripColor(int[] RGB) {
    setColor(LED.SIDE_STRIP_HEIGHT, length - LED.SIDE_STRIP_HEIGHT, RGB);
  }

  private static void setRainbow(int start, int stop) {
    for (int pixel = start; pixel < stop; pixel++) {
      // int[] rgb = HCLtoRGB(new double[] {(pixel / 100.0 + time * 1.0) % 1.0, 0.3, 0.6});
      // setColor(pixel, rgb);
    }
  }

  private static void setColorFlash(int start, int stop, int[] RGB, double speed) {
    double time = Timer.getFPGATimestamp();

    double val = (time * speed) % 1.0;
    if (val < 0.5) {
      setColor(0, length, RGB);
    } else {
      setColor(0, length, new int[] {0, 0, 0});
    }
  }

  // setColorWave(length - Constants.LED.SIDE_STRIP_HEIGHT, length, RSL_ORANGE, 1, Direction.RIGHT);
  private static void setColorWave(int start, int stop, int[] RGB, double speed, Direction dir) {
    setColorWave(start, stop, RGB, new int[] {0, 0, 0}, speed, dir);
  }

  private static void setColorWave(
      int start, int stop, int[] firstRGB, int[] secondRGB, double speed, Direction dir) {
    for (int pixel = 0; pixel < stop - start; pixel++) {
      double t = (pixel / 50.0 + time * speed) % 1.0;

      int[] interpolatedRGB = new int[3];
      for (int i = 0; i < 3; i++) {
        interpolatedRGB[i] = (int) (firstRGB[i] * (1 - t) + secondRGB[i] * t);
      }

      if (dir == Direction.LEFT) {
        setColor(pixel + start, interpolatedRGB);
      } else if (dir == Direction.RIGHT) {
        setColor(stop - pixel - 1, interpolatedRGB);
      }
    }
  }

  private static void setColorBounce(int start, int stop, int[] RGB, double speed) {
    int length = stop - start;

    // Calculate the current pixel position
    int pos = (int) ((((Timer.getFPGATimestamp() * speed)) * 2 * length) % (2.0 * length));

    pos = (pos < length) ? pos : 2 * length - pos - 1;

    // Set all pixels to off
    for (int pixel = start; pixel < stop; pixel++) {
      setColor(pixel, new int[] {0, 0, 0});
    }

    for (int pixel = Math.max(0, pos - 10); pixel < Math.min(stop, pos + 10); pixel++) {
      setColor(pixel, RGB);
    }
  }

  private static void setColorFromCenter(int start, int stop, int[] RGB, double speed) {
    centerFillCalled = true;
    for (int pixel = (int) Math.max(0, length / 2 - (length / 2 * (centerFillTimer / speed)));
        pixel < (int) Math.min(length, length / 2 + (length / 2 * (centerFillTimer / speed)));
        pixel++) {
      setColor(pixel, RGB);
    }
  }

  // private static void setGradientWave(int start, int stop, int[] firstRGB, int[] secondRGB,
  // double speed) {
  //   double time = Timer.getFPGATimestamp();
  //   int numLEDs = stop - start;
  //   double maxPoint = (40 + time * 100) % numLEDs; // Pixel index with second rgb

  //   double rDiff = (secondRGB[0] - firstRGB[0]);
  //   double gDiff = (secondRGB[1] - firstRGB[1]);
  //   double bDiff = (secondRGB[2] - firstRGB[2]);

  //   GradientTools.makeGradient(start, stop, numLEDs);

  //   for (int pixel = start; pixel < stop; pixel ++) {
  //     // int r = (int)(firstRGB[0] + rStep * (pixel - start));
  //     // int g = (int)(firstRGB[1] + gStep * (pixel - start));
  //     // int b = (int)(firstRGB[2] + bStep * (pixel - start));
  //     int[] newColor = firstRGB;
  //     newColor[0] = (int) (((newColor[0] + time * 50) * rDiff / 500 + pixel * 10) % 255);
  //     newColor[1] = (int) (((newColor[1] + time * 50) * gDiff / 500 + pixel * 10) % 255);
  //     newColor[2] = (int) (((newColor[2] + time * 50) * bDiff / 500 + pixel * 10) % 255);

  //     setColor(pixel, newColor);
  //   }

  // }

  private static int[] getBumperLEDColor() {
    if (TEAM_COLOR.IS_BLUE_TEAM.get()) {
      return BLUE;
    } else {
      return RED;
    }
  }

  private static void setBumperColorWaveWithPurple(int start, int stop, double speed) {
    if (TEAM_COLOR.IS_BLUE_TEAM.get()) {
      setColorWave(start, stop, getBumperLEDColor(), PURPLE, speed, Direction.LEFT);
    } else {
      setColorWave(start, stop, getBumperLEDColor(), PURPLE, speed, Direction.LEFT);
    }
  }

  private static void setBumperColorWave(int start, int stop, double speed) {
    if (TEAM_COLOR.IS_BLUE_TEAM.get()) {
      setColorWave(start, stop, getBumperLEDColor(), speed, Direction.LEFT);
    } else {
      setColorWave(start, stop, getBumperLEDColor(), speed, Direction.LEFT);
    }
  }

  // private static void setBumperGradientWave(int start, int stop) {
  //   if (Constants.IS_BLUE_TEAM) {
  //     setGradientWave(start, stop, getBumperColor(), new int[] {179, 0, 255}, 2.5);
  //   } else {
  //     setGradientWave(start, stop, RED,  getBumperColor(), 2.5);
  //   }

  // }

  // private static void setAcceleratingColorWav

  private static void clear() {
    setColor(0, length, new int[] {0, 0, 0});
  }

  // private static int[] HCLtoRGB(double[] HCL) {
  //   float OKLAB = ColorTools.oklabByHCL((float) HCL[0], (float) HCL[1], (float) HCL[2], (float)
  // 1.0);
  //   return new int[] {ColorTools.redInt(OKLAB), ColorTools.greenInt(OKLAB),
  // ColorTools.blueInt(OKLAB)};
  // }
}
