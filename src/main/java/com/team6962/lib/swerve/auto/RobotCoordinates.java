package com.team6962.lib.swerve.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface RobotCoordinates {
  public static Rotation2d HALF = Rotation2d.fromDegrees(180);

  public static Optional<Boolean> isAllianceInverted() {
    return DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red);
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
