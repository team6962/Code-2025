package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team6962.lib.swerve.SwerveConfig.Chassis;
import com.team6962.lib.swerve.module.SwerveModule.Corner;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class ModuleCornerTest {
  private void moduleCornerRotation(double angle, Corner corner) {
    assertEquals(angle, corner.getModuleRotation().in(Rotations), 0.0001);
  }

  @Test
  public void frontLeftCornerRotation() {
    moduleCornerRotation(0, SwerveModule.Corner.FRONT_LEFT);
  }

  @Test
  public void frontRightCornerRotation() {
    moduleCornerRotation(0.75, SwerveModule.Corner.FRONT_RIGHT);
  }

  @Test
  public void backLeftCornerRotation() {
    moduleCornerRotation(0.25, SwerveModule.Corner.BACK_LEFT);
  }

  @Test
  public void backRightCornerRotation() {
    moduleCornerRotation(0.5, SwerveModule.Corner.BACK_RIGHT);
  }

  private void moduleCornerRelativeTransform(Translation2d expected, Corner corner) {
    Translation2d actual = SwerveModule.calculateRelativeTranslation(corner.index, testChassis);

    assertEquals(expected.getX(), actual.getX(), 0.0001);
    assertEquals(expected.getY(), actual.getY(), 0.0001);
  }

  /*                                                                   wheelBase                  trackWidth */
  Chassis testChassis = new Chassis(null, null, Meters.of(10.4), Meters.of(33.4), null);

  @Test
  public void frontLeftCornerRelativeTransform() {
    moduleCornerRelativeTransform(new Translation2d(5.2, 16.7), SwerveModule.Corner.FRONT_LEFT);
  }

  @Test
  public void frontRightCornerRelativeTransform() {
    moduleCornerRelativeTransform(new Translation2d(5.2, -16.7), SwerveModule.Corner.FRONT_RIGHT);
  }

  @Test
  public void backLeftCornerRelativeTransform() {
    moduleCornerRelativeTransform(new Translation2d(-5.2, 16.7), SwerveModule.Corner.BACK_LEFT);
  }

  @Test
  public void backRightCornerRelativeTransform() {
    moduleCornerRelativeTransform(new Translation2d(-5.2, -16.7), SwerveModule.Corner.BACK_RIGHT);
  }
}
