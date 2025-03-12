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
  private static State state = State.AIMING_BARGE;
  //private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  private static final Distance kLedSpacing = Meters.of(1 / 60.0);


  public static enum State {
    OFF,
    DISABLED,
    ENABLED,
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
     SCORING_REEF,
     HANG
  }

  public static final Color WHITE = new Color(255, 255, 255);
  public static final Color RED = new Color(0,255 , 0);
  public static final Color GREEN = new Color(255, 0, 0);
  public static final Color BLUE = new Color(0, 0, 255);
  public static final Color RSL_ORANGE = new Color(100, 255, 0);
  public static final Color LIGHT_BLUE = new Color(216, 173, 230);
  public static final Color YELLOW = new Color(255, 255, 0);
  public static final Color CYAN = new Color(255, 0, 255);
  public static final Color DARK_GREEN = new Color(50, 0, 0);
  public static final Color PURPLE = new Color(59, 108, 170);
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
   
    LEDPattern m_rainbow = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, ColorFrom, ColorTo);
    //LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern blink = LEDPattern.solid(WHITE).blink(Seconds.of(Blink));
    LEDPattern scroll = m_rainbow.scrollAtRelativeSpeed(Percent.per(Second).of(Scroll));
    //LEDPattern scroll = base.scrollAtRelativeSpeed(Percent.per(Second).of(Scroll));

    LEDPattern pattern = scroll.mask(blink);

    return pattern;
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
      case OFF:
        apply(createColor(new Color(0, 0, 0), new Color(0, 0, 0), 0.0, 0.0));
        break;
      case DISABLED:
        break;
      case ENABLED:
        apply(createColor(GREEN, DARK_GREEN, 0.0, 50.0));
        break;
      case DRIVING_AUTO:
        apply(createColor(WHITE, WHITE, 0.0, 50.0));
        break;
      case DRIVING_TELEOP_RED:
        apply(createColor(RED, YELLOW, 0.0, 50.0));
        break;
      case DRIVING_TELEOP_BLUE:
        apply(createColor(BLUE, LIGHT_BLUE, 0.0, 50.0));
        break;
      case HAS_ALGAE:
        apply(createColor(RED, YELLOW, 0.0, 0.0));
        break;
      case HAS_CORAL:
        break;
      case CAN_SEE_ALGAE:
        apply(createColor(CYAN, CYAN, 0.0, 0.0));
        break;
      case AIMING_PROCESSOR:
        apply(createColor(DARK_GREEN, DARK_GREEN, 0.5, 0.0));
        break;
      case SCORING_PROCESSOR:
        apply(createColor(DARK_GREEN, DARK_GREEN, 0.0, 0.0));
        break;
      case AIMING_BARGE:
        apply(createColor(YELLOW, YELLOW, 0.5, 0.0));
        break;
      case SCORING_BARGE:
        apply(createColor(YELLOW, YELLOW, 0.0, 0.0));
        break;
      case AIMING_REEF:
        apply(createColor(MAGENTA, MAGENTA, 0.5, 0.0));
        break;
      case SCORING_REEF:
        apply(createColor(MAGENTA, MAGENTA, 0.0, 0.0));
        break;
      case HANG:
        apply(createColor(RSL_ORANGE, RSL_ORANGE, 0.0, 0.0));
        break;


    }
  }
}
