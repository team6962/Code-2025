// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Constants.TEAM_COLOR;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

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
  public static final double WIDTH = FIELD_LAYOUT.getFieldWidth(); // meters
  public static final double LENGTH = FIELD_LAYOUT.getFieldLength(); // meters

  private static final Pose2d RIGHT_CORAL_STATION = new Pose2d(Inches.of(33.289704 + 20), Inches.of(25.533630 + 20), Rotation2d.fromDegrees(144.011392 + 90));

  public static enum CoralStation {
    LEFT(new Pose2d(RIGHT_CORAL_STATION.getX(), WIDTH - RIGHT_CORAL_STATION.getY(), RIGHT_CORAL_STATION.getRotation().unaryMinus())),
    RIGHT(RIGHT_CORAL_STATION);

    public final Pose2d pose;

    CoralStation(Pose2d pose) {
      this.pose = pose;
    }
  }

  public static enum Pole {
    LEFT,
    RIGHT
  }

  public static final Optional<Pose3d> getTagPose(int tagID) {
    return FIELD_LAYOUT.getTagPose(tagID);
  }

  public static final List<Supplier<Translation2d>> REEF_FACES = List.of();

  public static List<Translation2d> getReefFacePositions() {
    List<Translation2d> positions = new ArrayList<Translation2d>();

    for (int i = 0; i < 6; i++) {
      final double reefRadius = 32.75;

      positions.add(
          new Translation2d(
              Meters.convertFrom(176.745 + Math.cos(i * (Math.PI / 3)) * reefRadius, Inches),
              Meters.convertFrom(158.5 + Math.sin(i * (Math.PI / 3)) * reefRadius, Inches)));
    }

    return positions;
  }

  public static final Distance COMMON_FACE_POLE_DISTANCE = Inches.of(12.94);

  public static final int LEFT_STATION_TAG = !TEAM_COLOR.IS_BLUE_TEAM.get() ? 1 : 13;
  public static final int RIGHT_STATION_TAG = !TEAM_COLOR.IS_BLUE_TEAM.get() ? 2 : 12;
  public static final int PROCESSOR_TAG = !TEAM_COLOR.IS_BLUE_TEAM.get() ? 3 : 16;
  public static final int BARGE_TAG = !TEAM_COLOR.IS_BLUE_TEAM.get() ? 5 : 14;

  private static List<Integer> reefAprilTags;

  public static List<Integer> getReefAprilTagsByFace() {
    if (reefAprilTags == null) {
      reefAprilTags = new ArrayList<Integer>();

      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
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

  public static final Pose2d getFacePose(int face) {
    Pose2d tagPose = getTagPose(getReefAprilTagsByFace().get(face)).get().toPose2d();
    // return tagPose.rotateBy(Rotation2d.fromDegrees(180));
    return tagPose;
  }

  public static final Pose2d getPolePose(int face, Pole pole) {
    return getFacePose(face).transformBy(reefPoleOffset(pole));
  }

  public static final Pose2d getProcessorPose() {
    return getTagPose(PROCESSOR_TAG).get().toPose2d();
  }

  public static final Pose2d getOurBargePose() {
    return getTagPose(BARGE_TAG).get().toPose2d();
  }

  public static final Pose2d getLeftCoralStationPose() {
    return getTagPose(LEFT_STATION_TAG).get().toPose2d();
  }

  public static final Pose2d getRightCoralStationPose() {
    return getTagPose(RIGHT_STATION_TAG).get().toPose2d();
  }

  private static List<Pose2d> getCoralPlacementPoses() {
    List<Pose2d> positions = new ArrayList<Pose2d>();

    for (int i = 0; i < 6; i++) {
      // All the numbers are in inches because that's what the field map uses
      final double shiftAngle =
          Math.atan2(6.46, 32.75); // The angle from the middle of one side and a reef pole
      final double reefPoleRadius =
          Math.hypot(6.46, 32.75); // Individual REEF distance from the center

      positions.add(
          new Pose2d(
              176.745
                  + Math.cos(i * (Math.PI / 3) + shiftAngle) * reefPoleRadius
                  + Math.cos(i * (Math.PI / 3)) * 20,
              158.5
                  + Math.sin(i * (Math.PI / 3) + shiftAngle) * reefPoleRadius
                  + Math.sin(i * (Math.PI / 3)) * 20,
              new Rotation2d(i * (Math.PI / 3) + Math.PI)));

      positions.add(
          new Pose2d(
              176.745
                  + Math.cos(i * (Math.PI / 3) - shiftAngle) * reefPoleRadius
                  + Math.cos(i * (Math.PI / 3)) * 20,
              158.5
                  + Math.sin(i * (Math.PI / 3) - shiftAngle) * reefPoleRadius
                  + Math.sin(i * (Math.PI / 3)) * 20,
              new Rotation2d(i * (Math.PI / 3) + Math.PI)));
    }

    return positions;
  }

  public static final List<Translation2d> REEF_FACE_POSITIONS = getReefFacePositions();

  public static final List<Pose2d> CORAL_PLACEMENT_POSES = getCoralPlacementPoses();

  public static final Transform2d reefPoleOffset(Pole pole) {
    if (pole == Pole.LEFT) {
      return new Transform2d(-COMMON_FACE_POLE_DISTANCE.in(Meters) / 2.0, 0, null);
    }
    return new Transform2d(COMMON_FACE_POLE_DISTANCE.in(Meters) / 2.0, 0, null);
  }

  public static Supplier<Pose2d> pose2d(double x, double y, double degrees) {
    return () -> flipIfRed(new Pose2d(x, y, Rotation2d.fromDegrees(degrees))).get();
  }

  public static Supplier<Translation2d> point2d(double x, double y) {
    return () -> flipIfRed(new Translation2d(x, y)).get();
  }

  public static Supplier<Translation3d> point3d(double x, double y, double z) {
    return () -> flipIfRed(new Translation3d(x, y, z)).get();
  }

  public static Supplier<Translation2d> flipIfRed(Translation2d position) {
    return () -> new Translation2d(flipIfRed(position.getX()).get(), position.getY());
  }

  public static Supplier<Translation3d> flipIfRed(Translation3d position) {
    return () ->
        new Translation3d(flipIfRed(position.getX()).get(), position.getY(), position.getZ());
  }

  public static Supplier<Pose2d> flipIfRed(Pose2d pose) {
    return () ->
        new Pose2d(
            flipIfRed(pose.getTranslation()).get(),
            TEAM_COLOR.IS_BLUE_TEAM.get()
                ? pose.getRotation()
                : Rotation2d.fromDegrees(-(pose.getRotation().getDegrees() + 90) - 90));
  }

  public static Supplier<Double> flipIfRed(double x) {
    return () -> TEAM_COLOR.IS_BLUE_TEAM.get() ? x : LENGTH - x;
  }
}
