// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import edu.wpi.first.hal.PortsJNI;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.vision.AprilTags;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
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
  public static final double WIDTH = 8.05; // meters
  public static final double LENGTH = 17.55; // meters

  public static enum Pole {
    LEFT,
    RIGHT
  }

  public static final Optional<Pose3d> getTagPose(int tagID) {
    return AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(tagID);
  }

  public static final Map<String, Supplier<Pose2d>> AUTO_MOVE_POSITIONS =
      Map.of(
          "AMP", pose2d(1.85, 7.5, 90.0),
          "SOURCE", pose2d(15.4, 1.0, -60.0),
          "SPEAKER", pose2d(1.5, 5.5, 180.0),
          "TRAP", pose2d(6, WIDTH / 2, 180.0));

  public static final List<Supplier<Translation2d>> NOTE_POSITIONS =
      List.of(
          point2d(Units.inchesToMeters(114), WIDTH / 2.0 + Units.inchesToMeters(57) * 2.0),
          point2d(Units.inchesToMeters(114), WIDTH / 2.0 + Units.inchesToMeters(57) * 1.0),
          point2d(Units.inchesToMeters(114), WIDTH / 2.0 + Units.inchesToMeters(57) * 0.0),
          point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * 2.0),
          point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * 1.0),
          point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * 0.0),
          point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * -1.0),
          point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * -2.0));


  public static List<Translation2d> getReefFacePositions() {
    List<Translation2d> positions = new ArrayList<Translation2d>();

    for (int i = 0; i < 6; i++) {
      final double reefRadius = 32.75;

      positions.add(
          new Translation2d(
              Meters.convertFrom(176.745 + Math.cos(i * (Math.PI / 3)) * reefRadius, Inches),
              Meters.convertFrom(158.5 + Math.sin(i * (Math.PI / 3)) * reefRadius, Inches)
          )
      );
    }

    return positions;
  }

  public static final Distance COMMON_FACE_POLE_DISTANCE = Inches.of(12.94);

 
  public static final int LEFT_STATION_TAG = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 1 : 13;
  public static final int RIGHT_STATION_TAG = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 2 : 12;
  public static final int PROCESSOR_TAG = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 3 : 16;
  public static final int BARGE_TAG = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 5 : 14;


 /**
   * reef tags keyed by their face
   */
  public static final List<Integer> REEF_APRILTAGS = new ArrayList<Integer>();
  static {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      REEF_APRILTAGS.add(10);
      REEF_APRILTAGS.add(9);
      REEF_APRILTAGS.add(8);
      REEF_APRILTAGS.add(7);
      REEF_APRILTAGS.add(6);
      REEF_APRILTAGS.add(11);
    } else {
      // default blue
      REEF_APRILTAGS.add(21);
      REEF_APRILTAGS.add(22);
      REEF_APRILTAGS.add(17);
      REEF_APRILTAGS.add(18);
      REEF_APRILTAGS.add(19);
      REEF_APRILTAGS.add(20);
    }
  };

  

  public static final Pose2d getFacePose(int face) {
    Pose2d tagPose = getTagPose(REEF_APRILTAGS.get(face)).get().toPose2d();
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
            Constants.IS_BLUE_TEAM.get()
                ? pose.getRotation()
                : Rotation2d.fromDegrees(-(pose.getRotation().getDegrees() + 90) - 90));
  }

  public static Supplier<Double> flipIfRed(double x) {
    return () -> Constants.IS_BLUE_TEAM.get() ? x : LENGTH - x;
  }
}
