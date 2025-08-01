package com.team6962.lib.swerve.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/**
 * A {@code SwerveGyroscope} provides a way to get and reset the robot's heading. When the robot is
 * in simulation, the gyroscope will use odometry to calculate the robot's heading. When running on
 * a real robot, the gyroscope will use the NavX gyro.
 *
 * <p><i>Note: Outside of {@link PoseEstimator}, the {@code SwerveGyroscope} should only be used to
 * reset the robot's heading. To get the robot's current heading, use {@link
 * PoseEstimator#getEstimatedPose()} or {@link SwerveDrive#getEstimatedPose()}.</i>
 */
public class SwerveGyroscope extends SubsystemBase {
  private Angle offset = Radians.of(0);
  private AHRS navx;
  private Supplier<SwerveModulePosition[]> moduleDeltasSupplier;
  private SwerveDriveKinematics kinematics;
  private Angle absoluteHeading = Radians.of(0);
  private AngularVelocity angularVelocity;

  public SwerveGyroscope(
      Supplier<SwerveModulePosition[]> moduleDeltasSupplier, SwerveDriveKinematics kinematics) {
    if (RobotBase.isReal()) {
      connectNavX();
    }

    this.moduleDeltasSupplier = moduleDeltasSupplier;
    this.kinematics = kinematics;

    setName("Swerve Drive/Gyroscope");

    Logger.logSwerveModulePositions(getName() + "/moduleDeltas", moduleDeltasSupplier);
    Logger.logMeasure(getName() + "/absoluteHeading", this::getAbsoluteHeading);
    Logger.logBoolean(getName() + "/connected", () -> navx != null && navx.isConnected());
  }

  private void connectNavX() {
    // Try to instantiate an AHRS instance that connects to the roboRIO's
    // builtin NavX gyroscope
    try {
      navx = new AHRS(NavXComType.kUSB2);
    } catch (RuntimeException e) {
      DriverStation.reportError("Failed to initialize NavX-MXP Gyroscope: " + e.getMessage(), true);
      System.err.print("Failed to initialize NavX-MXP Gyroscope: ");
      e.printStackTrace();
    }

    Logger.logMeasure(getName() + "/angularVelocity", () -> angularVelocity);
    Logger.logMeasure(getName() + "/continuousYaw", () -> Degrees.of(navx.getAngle()));
    Logger.logMeasure(getName() + "/Gyroscope/discontinuousYaw", () -> Degrees.of(navx.getYaw()));
    Logger.logMeasure(
        getName() + "/Gyroscope/discontinuousPitch", () -> Degrees.of(navx.getPitch()));
    Logger.logMeasure(getName() + "/Gyroscope/discontinuousRoll", () -> Degrees.of(navx.getRoll()));
    Logger.logMeasure(
        getName() + "/Gyroscope/linearAccelX", () -> Degrees.of(navx.getWorldLinearAccelX()));
    Logger.logMeasure(
        getName() + "/Gyroscope/linearAccelY", () -> Degrees.of(navx.getWorldLinearAccelY()));
    Logger.logMeasure(
        getName() + "/Gyroscope/linearAccelZ", () -> Degrees.of(navx.getWorldLinearAccelZ()));
  }

  @Override
  public void periodic() {
    if (RobotBase.isReal() && navx != null && navx.isConnected() && !navx.isCalibrating()) {
      absoluteHeading = Degrees.of(navx.getAngle()).times(-1);
      angularVelocity = DegreesPerSecond.of(navx.getRate()).times(-1);
    } else {
      Angle headingChange = Radians.of(kinematics.toTwist2d(moduleDeltasSupplier.get()).dtheta);

      Logger.log(getName() + "/headingChange", headingChange);

      if (Math.abs(headingChange.in(Radians)) < 0.01) {
        headingChange = Radians.of(0);
      }

      absoluteHeading = absoluteHeading.plus(headingChange);
    }
  }

  /**
   * Get the NavX gyro object. This method will return null if the robot is not running on a real
   * robot.
   *
   * @return The {@link AHRS} gyro object
   */
  public AHRS getNavX() {
    return navx;
  }

  /**
   * Get the absolute heading of the robot. This is the robot's heading relative to it's starting
   * position.
   *
   * @return
   */
  public Angle getAbsoluteHeading() {
    return absoluteHeading;
  }

  public AngularVelocity getAngularVelocity() {
    return angularVelocity;
  }

  /**
   * Get the robot's current heading. This is the robot's heading relative to the starting position,
   * with the offset applied.
   *
   * @return The robot's current heading
   */
  public Angle getHeading() {
    return getAbsoluteHeading().plus(offset);
  }

  /** Reset the robot's heading to zero. */
  public void resetHeading() {
    offset = getAbsoluteHeading().unaryMinus();
  }

  /**
   * Set the robot's heading to a specific angle.
   *
   * @param angle The angle to set the robot's heading to
   */
  public void setHeading(Angle angle) {
    Angle absoluteAngle = getAbsoluteHeading();

    offset = angle.minus(absoluteAngle);
  }
}
