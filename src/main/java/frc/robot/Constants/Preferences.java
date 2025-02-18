// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Preferences {

  public static final class HANG_PIVOT {}

  public static final class HANG { // Adjust these as needed
    public static final Current MAX_CURRENT = Amps.of(80);
    public static final PIDConstants DEPLOY_PROFILE = new PIDConstants(1.0, 0.0, 0.0);

    public static final Angle ENCODER_OFFSET = Radians.of(0.0);

    public static final Angle HANG_ANGLE = Degrees.of(180.0);
    public static final Angle SWAP_ANGLE = Degrees.of(80.0);
    public static final Angle DEPLOY_ANGLE = Degrees.of(0.0);
    public static final Angle STOW_ANGLE = Degrees.of(110.0);
  }

  public static final class MANIPULATOR {
    public static final boolean INVERT_ALGAE_LEFT = true;
    public static final boolean INVERT_ALGAE_RIGHT = true;

    public static final Current ALGAE_DETECT_CURRENT = Amps.of(15);
    public static final Time ALGAE_GRIP_CHECK_TIME = Seconds.of(0.25);
    public static final Time ALGAE_GRIP_CHECK_RATE = Seconds.of(5.0);
    public static final boolean ALGAE_GRIP_CHECK_ENABLED = true;

    public static final double ALGAE_OUT_SPEED = -0.2;
    public static final double ALGAE_IN_SPEED = 0.2;
    public static final double ALGAE_HOLD_SPEED = 0.1;

    public static final double ALGAE_GRIP_CHECK_SPEED = 0.1;
    public static final double CORAL_OUT_SPEED = 0.4;
    public static final double CORAL_IN_SPEED = 0.2;
    public static final double CORAL_HOLD_SPEED = -0.0;
  }

  // REFERENCE ANGLES FROM CAD AND
  // https://docs.google.com/spreadsheets/d/1nObnDdU-mXogmLKZjKTFBk0GvdE3hzZU7_AcbApUQno/edit?gid=0#gid=0
  public static final class MANIPULATOR_PIVOT {}

  public static final class ELEVATOR {}
}
