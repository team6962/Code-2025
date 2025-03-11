package com.team6962.lib.swerve.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Field;
import frc.robot.util.CachedRobotState;
import java.util.Optional;

public interface RobotCoordinates {
  public static Rotation2d HALF = Rotation2d.fromDegrees(180);
  public static Translation2d FIELD_CENTER = new Translation2d(Field.WIDTH, Field.LENGTH).div(2);

  public static Optional<Boolean> isAllianceInverted() {
    return CachedRobotState.isAllianceInverted();
  }

  public static Pose2d visionToFieldPose(Pose2d visionPose) {
    if (isAllianceInverted().orElse(false)) {
      return new Pose2d(
          visionPose.getTranslation().rotateAround(FIELD_CENTER, HALF),
          visionPose.getRotation().plus(HALF));
    } else {
      return visionPose;
    }
  }

  public static Rotation2d visionToFieldRotation(Rotation2d visionRotation) {
    if (isAllianceInverted().orElse(false)) {
      return visionRotation.plus(HALF);
    } else {
      return visionRotation;
    }
  }

  public Pose2d getEstimatedPose();

  public default Rotation2d getEstimatedHeading() {
    return getEstimatedPose().getRotation();
  }

  public default ChassisSpeeds fieldToRobot(ChassisSpeeds fieldVelocity) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldVelocity, getEstimatedHeading());
  }

  public default ChassisSpeeds robotToField(ChassisSpeeds robotVelocity) {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getEstimatedHeading());
  }

  public default Translation2d fieldToRobot(Translation2d fieldVelocity) {
    return fieldVelocity.rotateBy(getEstimatedHeading().unaryMinus());
  }

  public default Translation2d robotToField(Translation2d robotVelocity) {
    return robotVelocity.rotateBy(getEstimatedHeading());
  }
}
