package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class LEDsTEST extends SubsystemBase {
  private static AddressableLED strip;
  private static AddressableLEDBuffer buffer;
  private RobotStateController stateController;
  private static int length = 60;
  private static State state = State.HAS_ALGAE;
  private static double time = 0;
  private static double centerFillTimer = 0;
  private static boolean centerFillCalled = false;

  public static enum State {
    OFF,
    DISABLED,
    ENABLED,
    CAN_SEE_ALGAE,
    HAS_ALGAE,
    HAS_VISION_TARGET_SPEAKER,
    RUNNING_COMMAND,
    AIMING,
    AIMING_IN_RANGE,
    AIMED,
    BAD,
    GOOD,
  }

  public static enum Direction {
    LEFT,
    RIGHT,
  }

  public static final Color WHITE = new Color(255, 255, 255);
  public static final int[] ANTARES_BLUE = {0, 0, 255};
  public static final int[] ANTARES_YELLOW = {255, 100, 0};
  public static final int[] RED = {255, 0, 0};
  public static final int[] RSL_ORANGE = {255, 100, 0};
  public static final int[] GREEN = {0, 255, 0};
  public static final int[] BLUE = {0, 20, 255};
  public static final int[] PURPLE = {100, 0, 255};

  private DoubleSupplier animationSpeed;

  public LEDsTEST(RobotStateController stateController, DoubleSupplier animationSpeed) {
    this.stateController = stateController;
    this.animationSpeed = animationSpeed;

    strip = new AddressableLED(9);
    buffer = new AddressableLEDBuffer(length);
    strip.setLength(buffer.getLength());

    strip.setData(buffer);
    strip.start();

    // setColorWave(0, length, PURPLE, ANTARES_BLUE, 1.0, Direction.LEFT);

    // setRainbow(0, length);

    strip.setData(buffer);

    state = State.HAS_ALGAE;

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
    System.out.println("State updated to: " + state);
    return Commands.run(() -> setState(state));
  }

  public static void setState(State state) {
    if (state.ordinal() > LEDsTEST.state.ordinal()) LEDsTEST.state = state;
  }

  private static void setColor(int pixel, int[] RGB) {
    buffer.setRGB(pixel, RGB[0], RGB[1], RGB[2]);
  }

  private static void setColor(int start, int stop, int[] RGB) {
    for (int pixel = start; pixel < stop; pixel++) {
      setColor(pixel, RGB);
    }
  }
}
