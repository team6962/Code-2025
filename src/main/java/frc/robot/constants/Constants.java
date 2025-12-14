// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // DEVICES
  public static final class DEVICES {
    public static final int DRIVE_XBOX_CONTROLLER = 0;
    public static final int OPERATOR_XBOX_CONTROLLER = 1;
  }

  public static final class SWERVE_DRIVE {
    public static final double TELEOPERATED_FINE_TUNE_DRIVE_POWER =
        0.1; // Percent driving power when using d-pad
    public static final double TELEOPERATED_DRIVE_POWER = 0.5; // Percent driving power
    public static final double TELEOPERATED_BOOST_POWER =
        1.0; // Percent power when using the triggers
    public static final double TELEOPERATED_ROTATE_POWER = 0.3; // Percent rotating power
    public static final double TELEOPERATED_ROTATE_BOOST_POWER = 0.5; // Percent rotating power
    public static final double TELEOPERATED_SUPER_ROTATE_POWER = 0.7; // Percent rotating power
    public static final double TELEOPERATED_ULTRA_FINE_TUNE_DRIVE_POWER = 0.5;
  }
}
