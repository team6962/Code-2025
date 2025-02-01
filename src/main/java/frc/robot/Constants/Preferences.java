// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Preferences {
  public static final class SWERVE_DRIVE {
    public static final double TELEOPERATED_FINE_TUNE_DRIVE_POWER =
        0.1; // Percent driving power when using d-pad
    public static final double TELEOPERATED_DRIVE_POWER = 0.5; // Percent driving power
    public static final double TELEOPERATED_BOOST_POWER =
        1.0; // Percent power when using the triggers
    public static final double TELEOPERATED_ROTATE_POWER = 0.5; // Percent rotating power
  }

  public static final class AMP_WHEELS {
    public static final double POWER = 1.0;
  }

  public static final class HANG_PIVOT {
    public static final Angle MAX_ANGLE = Degrees.of(90.0);
    public static final Angle MIN_ANGLE = Degrees.of(-60.0);
    public static final Angle STOW_ANGLE = Degrees.of(68.0);
    public static final Angle HANG_ANGLE = Degrees.of(-50.0);
  }

  public static final class HANG { // Adjust these as needed
    public static final double CLIMB_POWER = 0.5;
    public static final double REVERSE_POWER = 0.5;
  }

  public static final class INTAKE {
    public static final double IN_POWER = 0.5;
    public static final double OUT_POWER = 0.5; // Placeholder value

    public static final Angle PIVOT_DOWN = Degrees.of(0.0);
    public static final Angle PIVOT_UP = Degrees.of(90.0);

    public static final double TO_SHOOTER_POWER = 0.3;
    public static final double SLOW_OUT_POWER = 0.4;
  }

  public static final class MANIPULATOR {
    public static final double ALGAE_OUT_SPEED = 1.0;
    public static final double ALGAE_IN_SPEED = -1.0;
    public static final double CORAL_OUT_SPEED = 1.0;
    public static final double CORAL_IN_SPEED = -1.0;
  }

  public static final class MANIPULATOR_PIVOT {
    public static final Angle MAX_ANGLE = Degrees.of(0.0);
    public static final Angle MIN_ANGLE = Degrees.of(0.0);
  }

  public static final class ELEVATOR {
    public static final Distance MAX_HEIGHT = Inches.of(80);
    public static final Distance MIN_HEIGHT = Inches.of(0);
    public static final Distance STOW_HEIGHT = Inches.of(0);
    public static final Distance L1_HEIGHT = Inches.of(0);
    public static final Distance L2_HEIGHT = Inches.of(0);
    public static final Distance L3_HEIGHT = Inches.of(0);
    public static final Distance L4_HEIGHT = Inches.of(0);
    public static final Distance BARGE_HEIGHT = Inches.of(0);
  }

  public static final class VOLTAGE_LADDER {
    public static final double SWERVE_DRIVE = 7.0;
    public static final double SHOOTER = 8.0;
    public static final double INTAKE = 8.5;
    public static final double MANIPULATOR = 8; // placeholder value
  }
}
