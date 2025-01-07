package com.team6962.lib.swerve.auto;

import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveGyroscope extends SubsystemBase {
    private Rotation2d offset = new Rotation2d();
    private AHRS navx;
    private Supplier<SwerveModulePosition[]> moduleDeltasSupplier;
    private SwerveDriveKinematics kinematics;
    private Rotation2d absoluteHeading = new Rotation2d();

    public SwerveGyroscope(Supplier<SwerveModulePosition[]> moduleDeltasSupplier, SwerveDriveKinematics kinematics) {
        if (RobotBase.isReal()) {
            connectNavX();
        }

        this.moduleDeltasSupplier = moduleDeltasSupplier;
        this.kinematics = kinematics;
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
            absoluteHeading = Rotation2d.fromDegrees(navx.getAngle());
        } else {
            Rotation2d headingChange = Rotation2d.fromRadians(kinematics.toTwist2d(moduleDeltasSupplier.get()).dtheta);

            absoluteHeading = absoluteHeading.plus(headingChange);
        }
    }

    public AHRS getNavX() {
        return navx;
    }

    public Rotation2d getAbsoluteHeading() {
        return absoluteHeading;
    }

    public Rotation2d getHeading() {
        return getAbsoluteHeading().plus(offset);
    }

    public void resetHeading() {
        if (navx != null) {
            navx.reset();
        } else {
            offset = Rotation2d.fromDegrees(-getAbsoluteHeading().getDegrees());
        }
    }

    public void setHeading(Rotation2d angle) {
        Rotation2d absoluteAngle = getAbsoluteHeading();

        offset = angle.minus(absoluteAngle);
    }
}
