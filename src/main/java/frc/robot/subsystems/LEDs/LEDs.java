// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.LED;
import frc.robot.subsystems.RobotStateController;

import com.team6962.lib.telemetry.Logger;

public class LEDs extends SubsystemBase {
  private static AddressableLED strip;
  private static AddressableLEDBuffer buffer;
  private RobotStateController stateController;
  private static State state = State.DRIVING_AUTO;


  public static enum State {
    DISABLED,
    DRIVING_AUTO,
    DRIVING_TELEOP,
    HAS_ALGAE,
    HAS_CORAL,
    DOUBLE_APRILTAG,
    AIMING
  }

  LEDPattern DRIVING_AUTO = createColor(RED, DARK_RED, 0.0, 50.0);
  LEDPattern DRIVING_TELEOP = createColor(BLUE, DARK_BLUE, 0.0, 50.0);
  LEDPattern DISABLED = LEDPattern.solid(MAGENTA);
  LEDPattern HAS_ALGAE = createColor(CYAN, CYAN, 0.5, 0.0);
  LEDPattern HAS_CORAL = createColor(MAGENTA, MAGENTA, 0.5, 0.0);
  LEDPattern DOUBLE_APRILTAG = LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(30));
  LEDPattern AIMING = createColor(RED, WHITE, 0.0, 50.0);

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

    if (DriverStation.getAlliance().equals(RED)) {
      DRIVING_AUTO = createColor(RED, DARK_RED, 0.0, 50.0);
      DRIVING_TELEOP = createColor(RED, WHITE, 0.0, 50.0);
    } else {
      DRIVING_AUTO = createColor(BLUE, DARK_BLUE, 0.0, 50.0);
      DRIVING_TELEOP = createColor(BLUE, WHITE, 0.0, 50.0);
    }
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
        apply(DISABLED);
        break;
      case DRIVING_AUTO:
        apply(DRIVING_AUTO);
        break;
      case DRIVING_TELEOP:
        apply(DRIVING_TELEOP);
        break;
      case AIMING:
        apply(AIMING);
        break;
      case DOUBLE_APRILTAG:
        apply(DOUBLE_APRILTAG);
        break;
      case HAS_ALGAE:
        apply(HAS_ALGAE);
        break;
      case HAS_CORAL:
        apply(HAS_CORAL);
        break;
    }
  }
}
