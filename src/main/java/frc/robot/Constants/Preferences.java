// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

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
  public static final class SWERVE_DRIVE {
    public static final double TELEOPERATED_FINE_TUNE_DRIVE_POWER =
        0.1; // Percent driving power when using d-pad
    public static final double TELEOPERATED_DRIVE_POWER = 0.5; // Percent driving power
    public static final double TELEOPERATED_BOOST_POWER =
        1.0; // Percent power when using the triggers
    public static final double TELEOPERATED_ROTATE_POWER = 0.5; // Percent rotating power
  }

  public static final class HANG_PIVOT {
    public static final Angle MAX_ANGLE = Degrees.of(140.0);
    public static final Angle MIN_ANGLE = Degrees.of(-40.0);
    public static final Angle STOW_ANGLE = Degrees.of(0.0);
    public static final Angle HANG_ANGLE = Degrees.of(-40.0);
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
    public static final boolean INVERT_ALGAE_LEFT = true;
    public static final boolean INVERT_ALGAE_RIGHT = true;

    public static final Current ALGAE_DETECT_CURRENT = Amps.of(10);
    public static final Time ALGAE_GRIP_CHECK_TIME = Seconds.of(0.25);
    public static final Time ALGAE_GRIP_CHECK_RATE = Seconds.of(5.0);
    public static final boolean ALGAE_GRIP_CHECK_ENABLED = true;

    public static final double ALGAE_OUT_SPEED = 0.1;
    public static final double ALGAE_IN_SPEED = 0.1;
    public static final double ALGAE_HOLD_SPEED = 0.05;
    public static final double CORAL_OUT_SPEED = 0.2;
    public static final double CORAL_IN_SPEED = 0.2;
    public static final double CORAL_HOLD_SPEED = -0.0; // -0.1
  }

  // REFERENCE ANGLES FROM CAD AND
  // https://docs.google.com/spreadsheets/d/1nObnDdU-mXogmLKZjKTFBk0GvdE3hzZU7_AcbApUQno/edit?gid=0#gid=0
  public static final class MANIPULATOR_PIVOT {
    public static final Angle MAX_ANGLE = Degrees.of(20.0); // RESET TO 40.0
    public static final Angle MIN_LOW_ANGLE = Degrees.of(-150.0);
    public static final Angle MIN_RAISED_ANGLE = Degrees.of(-150.0);
    public static final Angle STOW_ANGLE = Degrees.of(38.5);
    public static final Angle SAFE_ANGLE = Degrees.of(-34.5);

    public static final class CORAL {
      public static final Angle L1_ANGLE = Degrees.of(22.5);
      public static final Angle L23_ANGLE = Degrees.of(-34.5);
      public static final Angle L4_ANGLE = Degrees.of(-57.0);
      public static final Angle INTAKE_ANGLE = Degrees.of(-138.0);
    }

    public static final class ALGAE {
      public static final Angle BARGE_ANGLE = Degrees.of(32.75);
      public static final Angle REEF_ANGLE = Degrees.of(0.0);
      public static final Angle GROUND_ANGLE = Degrees.of(-35.3);
      public static final Angle PROCESSOR_ANGLE = Degrees.of(-10.0);
    }
  }

  public static final class ELEVATOR {
    // HEIGHT IS MEASURED FROM THE GROUND TO THE TOP OF THE ELEVATOR
    public static final Distance BASE_HEIGHT = Inches.of(35.5);
    public static final Distance MAX_HEIGHT = Inches.of(80);
    public static final Distance MIN_HEIGHT = BASE_HEIGHT;
    public static final Distance STOW_HEIGHT = BASE_HEIGHT;
    public static final Distance MAX_UNLIMITED_HEIGHT = Inches.of(41.0); // AVERAGE

    public static final class CORAL {
      public static final Distance L1_HEIGHT = Inches.of(39.0);
      public static final Distance L2_HEIGHT = Inches.of(49.8);
      public static final Distance L3_HEIGHT = Inches.of(58.6);
      public static final Distance L4_HEIGHT = Inches.of(77.2);
      public static final Distance INTAKE_HEIGHT = Inches.of(59.1);
    }

    public static final class ALGAE {
      public static final Distance L2_HEIGHT = Inches.of(0);
      public static final Distance L3_HEIGHT = Inches.of(0);
      public static final Distance BARGE_HEIGHT = MAX_HEIGHT;
      public static final Distance GROUND_HEIGHT = MIN_HEIGHT;
      public static final Distance PROCESSOR_HEIGHT = MIN_HEIGHT;
    }
  }

  public static final class VOLTAGE_LADDER {
    public static final double SWERVE_DRIVE = 7.0;
    public static final double ELEVATOR = 7.5;
    public static final double MANIPULATOR = 8.0;
    public static final double INTAKE = 8.5;
    public static final double HANG = 9.0;
  }
}
