package com.team6962.lib.utils;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.module.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public final class KinematicsUtils {
  private KinematicsUtils() {}

  public static SwerveModulePosition toModulePosition(SwerveModuleState state, Time delta) {
    double deltaSeconds = delta.in(Seconds);

    return new SwerveModulePosition(
        state.speedMetersPerSecond * deltaSeconds, state.angle.times(deltaSeconds));
  }

  public static SwerveModulePosition[] toModulePositions(SwerveModuleState[] states, Time delta) {
    SwerveModulePosition[] positions = new SwerveModulePosition[states.length];

    for (int i = 0; i < states.length; i++) {
      positions[i] = toModulePosition(states[i], delta);
    }

    return positions;
  }

  public static SwerveModuleState toModuleState(SwerveModulePosition change, Time delta) {
    double deltaSeconds = delta.in(Seconds);

    return new SwerveModuleState(
        change.distanceMeters / deltaSeconds, change.angle.div(deltaSeconds));
  }

  public static SwerveModuleState[] toModuleStates(SwerveModulePosition[] changes, Time delta) {
    SwerveModuleState[] states = new SwerveModuleState[changes.length];

    for (int i = 0; i < changes.length; i++) {
      states[i] = toModuleState(changes[i], delta);
    }

    return states;
  }

  public static SwerveModulePosition[] difference(
      SwerveModulePosition[] newPositions, SwerveModulePosition[] oldPositions) {
    SwerveModulePosition[] differences = new SwerveModulePosition[newPositions.length];

    for (int i = 0; i < newPositions.length; i++) {
      differences[i] =
          new SwerveModulePosition(
              newPositions[i].distanceMeters - oldPositions[i].distanceMeters,
              newPositions[i].angle);
    }

    return differences;
  }

  public static SwerveModulePosition[] multiply(SwerveModulePosition[] positions, double scalar) {
    SwerveModulePosition[] scaled = new SwerveModulePosition[positions.length];

    for (int i = 0; i < positions.length; i++) {
      scaled[i] =
          new SwerveModulePosition(positions[i].distanceMeters * scalar, positions[i].angle);
    }

    return scaled;
  }

  public static Translation2d[] modulePositionsFromChassis(SwerveConfig.Chassis chassis) {
    Translation2d[] translations = new Translation2d[4];

    for (int i = 0; i < 4; i++) {
      translations[i] = SwerveModule.calculateRelativeTranslation(i, chassis);
    }

    return translations;
  }

  public static SwerveDriveKinematics kinematicsFromChassis(SwerveConfig.Chassis chassis) {
    return new SwerveDriveKinematics(modulePositionsFromChassis(chassis));
  }

  public static Translation2d getTranslation(ChassisSpeeds speeds) {
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public static Rotation2d getRotation(ChassisSpeeds speeds) {
    return Rotation2d.fromRadians(speeds.omegaRadiansPerSecond);
  }

  public static SwerveModuleState[] desaturateWheelSpeeds(
      SwerveModuleState[] states, LinearVelocity maxSpeed) {
    double fraction = 1.0;

    for (SwerveModuleState state : states) {
      LinearVelocity moduleSpeed = MetersPerSecond.of(Math.abs(state.speedMetersPerSecond));

      fraction = Math.min(fraction, maxSpeed.in(MetersPerSecond) / moduleSpeed.in(MetersPerSecond));
    }

    SwerveModuleState[] limitedStates = new SwerveModuleState[states.length];

    for (int i = 0; i < states.length; i++) {
      limitedStates[i] =
          new SwerveModuleState(states[i].speedMetersPerSecond * fraction, states[i].angle);
    }

    return limitedStates;
  }

  public static SwerveModuleState[] getStoppedStates(SwerveModuleState[] currentStates) {
    SwerveModuleState[] output = new SwerveModuleState[currentStates.length];

    for (int i = 0; i < currentStates.length; i++) {
      output[i] = new SwerveModuleState(0, currentStates[i].angle);
    }

    return output;
  }

  public static SwerveModuleState[] getParkedStates() {
    SwerveModuleState[] output = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      output[i] =
          new SwerveModuleState(
              0,
              Rotation2d.fromDegrees(45)
                  .plus(new Rotation2d(SwerveModule.Corner.fromIndex(i).getModuleRotation())));
    }

    return output;
  }

  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  public static ChassisSpeeds allianceInvertSpeeds(ChassisSpeeds speeds) {
    return new ChassisSpeeds(
        -speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public static ChassisSpeeds rotateSpeeds(ChassisSpeeds speeds, Rotation2d angle) {
    Translation2d translation =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(angle);

    return new ChassisSpeeds(translation.getX(), translation.getY(), speeds.omegaRadiansPerSecond);
  }

  public static ChassisSpeeds createChassisSpeeds(Translation2d translation, Rotation2d rotation) {
    if (translation == null) {
      translation = new Translation2d(0, 0);
    }

    if (rotation == null) {
      rotation = new Rotation2d(0);
    }

    return new ChassisSpeeds(translation.getX(), translation.getY(), rotation.getRadians());
  }

  public static ChassisSpeeds setTranslationSpeed(
      ChassisSpeeds speeds, Translation2d translationSpeeds) {
    if (speeds == null) speeds = new ChassisSpeeds();

    return new ChassisSpeeds(
        translationSpeeds.getX(), translationSpeeds.getY(), speeds.omegaRadiansPerSecond);
  }

  public static ChassisSpeeds setRotationSpeed(ChassisSpeeds speeds, Rotation2d rotationSpeed) {
    if (speeds == null) speeds = new ChassisSpeeds();

    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, rotationSpeed.getRadians());
  }

  public static SwerveModuleState[] blankModuleStates(int count) {
    SwerveModuleState[] states = new SwerveModuleState[count];

    for (int i = 0; i < count; i++) {
      states[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    return states;
  }

  public static SwerveModulePosition[] blankModulePositions(int count) {
    SwerveModulePosition[] positions = new SwerveModulePosition[count];

    for (int i = 0; i < count; i++) {
      positions[i] = new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
    }

    return positions;
  }

  public static Rotation2d getAngle(double x, double y) {
    Translation2d translation = new Translation2d(x, y);

    if (translation.getNorm() <= 1e-6) {
      return Rotation2d.fromRotations(0);
    }

    return translation.getAngle();
  }
}
