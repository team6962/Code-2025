package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team6962.lib.swerve.auto.Coordinates;
import com.team6962.lib.utils.KinematicsUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import org.junit.jupiter.api.Test;

public class KinematicsTest {
  @Test
  public void desaturateWheelSpeeds() {
    LinearVelocity maxWheelSpeed = MetersPerSecond.of(2.0);
    SwerveModuleState[] states =
        new SwerveModuleState[] {
          new SwerveModuleState(MetersPerSecond.of(2.6), Rotation2d.fromDegrees(18)),
          new SwerveModuleState(MetersPerSecond.of(4.3), Rotation2d.fromDegrees(-42))
        };

    SwerveModuleState[] output = KinematicsUtils.desaturateWheelSpeeds(states, maxWheelSpeed);

    assertEquals(1.2093, output[0].speedMetersPerSecond, 0.001);
    assertEquals(2.0, output[1].speedMetersPerSecond, 0.001);
    assertEquals(18, output[0].angle.getDegrees(), 0.001);
    assertEquals(-42, output[1].angle.getDegrees(), 0.001);
  }

  private Coordinates createCoordinates(Pose2d pose) {
    return new Coordinates() {
      @Override
      public Pose2d getEstimatedPose() {
        return pose;
      }
    };
  }

  @Test
  public void allianceToRobotSpeeds() {
    Coordinates coordinates = createCoordinates(new Pose2d(2.5, -18.1, Rotation2d.fromDegrees(90)));
    ChassisSpeeds orginalSpeeds =
        new ChassisSpeeds(
            MetersPerSecond.of(1.0), MetersPerSecond.of(2.0), RotationsPerSecond.of(0.3));

    assertEquals(
        coordinates.robotToAllianceSpeeds(coordinates.allianceToRobotSpeeds(orginalSpeeds)),
        orginalSpeeds);
  }
}
