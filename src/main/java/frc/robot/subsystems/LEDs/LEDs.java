// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.LED;
import frc.robot.subsystems.RobotStateController;
import edu.wpi.first.units.measure.Distance;
import java.util.Locale;

import com.team6962.lib.telemetry.Logger;

public class LEDs extends SubsystemBase {
  private static AddressableLED strip;
  private static AddressableLEDBuffer buffer;
  private RobotStateController stateController;
  private static State state = State.DOUBLE_APRILTAG;
  //private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  private static final Distance kLedSpacing = Meters.of(1 / 60.0);


  public static enum State {
    DISABLED,
    DRIVING_AUTO,
    DRIVING_TELEOP,
    HAS_ALGAE,
    HAS_CORAL,
    DOUBLE_APRILTAG,
    AIMING
  }

  public static final Color WHITE = new Color(255, 255, 255);
  public static final Color RED = new Color(0,255 , 0);
  public static final Color DARK_RED = new Color(0,50 , 0);
  public static final Color GREEN = new Color(255, 0, 0);
  public static final Color BLUE = new Color(0, 0, 255);
  public static final Color DARK_BLUE = new Color(0, 0, 50);
  public static final Color CYAN = new Color(255, 0, 255);
  public static final Color MAGENTA = new Color(0, 255, 255);
  
  public LEDs(RobotStateController stateController) {
    this.stateController = stateController;

    strip = new AddressableLED(LED.port);
    buffer = new AddressableLEDBuffer(LED.length);
    strip.setLength(buffer.getLength());

    strip.setData(buffer);
    strip.start();
  }

  public LEDs(RobotStateController stateController2, Object object) {
    //TODO Auto-generated constructor stub
}

public static Command setStateCommand(State state) {
    return Commands.run(() -> setState(state));
  }

  public static void setState(State state) {
    if (state.ordinal() > LEDs.state.ordinal()) LEDs.state = state;
  }

  private static LEDPattern createColor(Color ColorFrom, Color ColorTo, double Blink, double Scroll) {
   
    LEDPattern pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, ColorFrom, ColorTo);
    LEDPattern l_blink = LEDPattern.solid(WHITE).blink(Seconds.of(Blink));
    LEDPattern l_scroll = pattern.scrollAtRelativeSpeed(Percent.per(Second).of(Scroll));

    return l_scroll.mask(l_blink);
  }
  
  private static void apply(LEDPattern pattern) {
    // Apply the LED pattern to the data buffer
    pattern.applyTo(buffer);

    // Write the data to the LED strip
    strip.setData(buffer);
  }

  @Override
  public void periodic() {
      Logger.log("LED", state.toString());
        switch (state) {
      case DISABLED:
        apply(LEDPattern.solid(MAGENTA));
        break;
      case DRIVING_AUTO:
        apply(createColor(WHITE, WHITE, 0.0, 50.0));
        break;
      case DRIVING_TELEOP:
        apply(createColor(RED, WHITE, 0.0, 50.0));
        break;
      case AIMING:
        apply(createColor(RED, WHITE, 0.0, 50.0));
        break;
      case DOUBLE_APRILTAG:
        apply(LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(30)));
        break;
      case HAS_ALGAE:
        apply(createColor(CYAN, CYAN, 0.5, 0.0));
        break;
      case HAS_CORAL:
        apply(createColor(MAGENTA, MAGENTA, 0.5, 0.0));
        break;
    }
  }
}
