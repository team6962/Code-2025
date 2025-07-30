// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.field;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.util.CachedRobotState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Field {
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static final double WIDTH = FIELD_LAYOUT.getFieldWidth(); // 8.052 meters
  public static final double LENGTH = FIELD_LAYOUT.getFieldLength(); // 17.548 meters

  private static List<Integer> reefAprilTags;

  public static List<Integer> getReefAprilTagsByFace() {
    if (reefAprilTags == null) {
      reefAprilTags = new ArrayList<Integer>();

      if (CachedRobotState.isRed().orElse(false)) {
        reefAprilTags.add(10);
        reefAprilTags.add(9);
        reefAprilTags.add(8);
        reefAprilTags.add(7);
        reefAprilTags.add(6);
        reefAprilTags.add(11);
      } else {
        reefAprilTags.add(21);
        reefAprilTags.add(22);
        reefAprilTags.add(17);
        reefAprilTags.add(18);
        reefAprilTags.add(19);
        reefAprilTags.add(20);
      }
    }

    return reefAprilTags;
  }
}
