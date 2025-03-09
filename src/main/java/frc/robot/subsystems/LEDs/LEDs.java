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
  private static State state = State.DRIVING_TELEOP_RED;
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
  public static final Color RED = new Color(255, 0, 0);
  public static final Color GREEN = new Color(0, 255, 0);
  public static final Color BLUE = new Color(0, 20, 255);
  public static final Color RSL_ORANGE = new Color(255, 100, 0);
  public static final Color LIGHT_BLUE = new Color(173, 216, 230);
  public static final Color YELLOW = new Color(255, 255, 0);
  public static final Color CYAN = new Color(0, 255, 255);
  public static final Color DARK_GREEN = new Color(0, 100, 0);
  public static final Color PURPLE = new Color(108, 59, 170);
  public static final Color MAGENTA = new Color(255, 0, 255);

  public static final LEDPattern OFF = createColor(new Color(0, 0, 0), new Color(0, 0, 0), 0.0, 0.0);
  public static final LEDPattern ENABLED = createColor(new Color(0, 0, 0), new Color(0, 0, 0), 0.0, 0.0);
  public static final LEDPattern DRIVING_AUTO = createColor(WHITE, WHITE, 0.0, 50.0);
  public static final LEDPattern DRIVING_TELEOP_RED = createColor(RED, YELLOW, 0.0, 50.0);
  public static final LEDPattern DRIVING_TELEOP_BLUE = createColor(BLUE, CYAN, 0.0, 50.0);
  public static final LEDPattern HAS_ALGAE = createColor(RED, YELLOW, 0.0, 0.0);
  public static final LEDPattern CAN_SEE_ALGAE = createColor(CYAN, CYAN, 0.0, 0.0);
  public static final LEDPattern AIMING_PROCESSOR = createColor(DARK_GREEN, DARK_GREEN, 0.5, 0.0);
  public static final LEDPattern SCORING_PROCESSOR = createColor(DARK_GREEN, DARK_GREEN, 0.0, 0.0);
  public static final LEDPattern AIMING_BARGE = createColor(YELLOW, YELLOW, 1.0, 0.0);
  public static final LEDPattern SCORING_BARGE = createColor(YELLOW, YELLOW, 0.0, 0.0);
  public static final LEDPattern AIMING_REEF = createColor(MAGENTA, MAGENTA, 0.5, 0.0);
  public static final LEDPattern SCORING_REEF = createColor(MAGENTA, MAGENTA, 0.0, 0.0);
  public static final LEDPattern HANG = createColor(RSL_ORANGE, RSL_ORANGE, 0.0, 0.0);
  
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
    //LEDPattern blink = m_rainbow.blink(Seconds.of(Blink));
    LEDPattern scroll = m_rainbow.scrollAtRelativeSpeed(Percent.per(Second).of(Scroll));
    //LEDPattern scroll = base.scrollAtRelativeSpeed(Percent.per(Second).of(Scroll));

    //LEDPattern pattern = blink.overlayOn(scroll);

    return scroll;
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
        apply(OFF);
        break;
      case DISABLED:
        break;
      case ENABLED:
        apply(ENABLED);
        break;
      case DRIVING_AUTO:
        apply(DRIVING_AUTO);
        break;
      case DRIVING_TELEOP_RED:
        Logger.log("LED", "C"); 
        apply(DRIVING_TELEOP_RED);
        break;
      case DRIVING_TELEOP_BLUE:
        apply(DRIVING_TELEOP_BLUE);
        break;
      case HAS_ALGAE:
        apply(HAS_ALGAE);
        break;
      case HAS_CORAL:
        break;
      case CAN_SEE_ALGAE:
        apply(CAN_SEE_ALGAE);
        break;
      case AIMING_PROCESSOR:
        apply(AIMING_PROCESSOR);
        break;
      case SCORING_PROCESSOR:
        apply(SCORING_PROCESSOR);
        break;
      case AIMING_BARGE:
        apply(AIMING_BARGE);
        break;
      case SCORING_BARGE:
        apply(SCORING_BARGE);
        break;
      case AIMING_REEF:
        apply(AIMING_REEF);
        break;
      case SCORING_REEF:
        apply(SCORING_REEF);
        break;
      case HANG:
        apply(HANG);
        break;
    }
  }
}
