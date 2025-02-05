package com.team6962.lib.swerve.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
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
  }

  private void connectNavX() {
    // Try to instantiate an AHRS instance that connects to the roboRIO's
    // builtin NavX gyroscope
    try {
      navx = new AHRS(NavXComType.kMXP_SPI);
    } catch (RuntimeException e) {
      DriverStation.reportError("Failed to initialize NavX-MXP Gyroscope: " + e.getMessage(), true);
      System.err.print("Failed to initialize NavX-MXP Gyroscope: ");
      e.printStackTrace();
    }

    // Reset the current heading to be the new zero when the NavX first receives heading data
    // registerCallback() may have been removed from the AHRS class
    // navx.registerCallback(new ITimestampedDataSubscriber() {
    //     @Override
    //     public void timestampedDataReceived(long system_timestamp, long sensor_timestamp,
    //             AHRSUpdateBase sensor_data, Object context) {
    //         SwerveGyroscope gyroscope = (SwerveGyroscope) context;

    //         gyroscope.resetHeading();
    //         gyroscope.getNavX().deregisterCallback(this);
    //     }
    // }, this);
  }

  @Override
  public void periodic() {
    if (RobotBase.isReal() && navx != null && navx.isConnected() && !navx.isCalibrating()) {
      absoluteHeading = Degrees.of(navx.getAngle()).times(-1);
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

  /**
   * Get the robot's current heading. This is the robot's heading relative to the starting position,
   * with the offset applied.
   *
   * @return The robot's current heading
   */
  public Angle getHeading() {
    return getAbsoluteHeading().plus(offset);
  }

  // h = a + o
  // 0 = a + o
  // o = -a

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
