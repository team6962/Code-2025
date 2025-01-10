package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.swerve.SwerveConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveModule extends Subsystem {
    public void configureModule(SwerveConfig config, Corner corner);

    public SwerveConfig getDrivetrainConstants();

    public default SwerveConfig.Module getModuleConstants() {
        return getDrivetrainConstants().module(getModuleCorner());
    }

    public Corner getModuleCorner();

    public void driveState(SwerveModuleState targetState);

    public Distance getDrivePosition();

    public LinearVelocity getDriveSpeed();

    public Angle getSteerAngle();

    public AngularVelocity getSteerVelocity();

    public SwerveModuleState getState();

    public SwerveModulePosition getPosition();

    public Current getConsumedCurrent();

    public default Pose2d getRelativePose() {
        return new Pose2d(
            SwerveModule.calculateRelativeTranslation(getModuleCorner().index, getDrivetrainConstants().chassis()),
            new Rotation2d(getSteerAngle())
        );
    }

    public Command calibrateSteerMotor(Voltage averageBusVoltage, Current maxCurrent);

    public Command calibrateDriveMotor(Voltage averageBusVoltage, Current maxCurrent);

    /**
     * Gets the name of a module given its index (e.g. 0 -> "Front Left").
     * @param moduleIndex The index of the module
     * @return The name of the module, formatted with spaces and capitalization
     */
    public static String getModuleName(int moduleIndex) {
        return switch (moduleIndex) {
            case 0 -> "Front Left";
            case 1 -> "Front Right";
            case 2 -> "Back Left";
            case 3 -> "Back Right";
            default -> throw new IllegalArgumentException("Invalid module index");
        };
    }

    /**
     * Gets the name of a module given its index (e.g. 0 -> "front-left").
     * @param moduleIndex The index of the module
     * @return The name of the module, formatted with hyphens and lowercase
     */
    public static String getModuleSysIdName(int moduleIndex) {
        return switch (moduleIndex) {
            case 0 -> "front-left";
            case 1 -> "front-right";
            case 2 -> "back-left";
            case 3 -> "back-right";
            default -> throw new IllegalArgumentException("Invalid module index");
        };
    }
    
    /**
     * Represents a corner of the robot that a module can be on.
     */
    public static enum Corner {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        BACK_LEFT(2),
        BACK_RIGHT(3);

        public final int index;

        private Corner(int index) {
            this.index = index;
        }

        /**
         * Converts an index to a {@link Corner} object.
         * @param index
         * @return
         */
        public static Corner fromIndex(int index) {
            return switch (index) {
                case 0 -> FRONT_LEFT;
                case 1 -> FRONT_RIGHT;
                case 2 -> BACK_LEFT;
                case 3 -> BACK_RIGHT;
                default -> throw new IllegalArgumentException("Invalid module index");
            };
        }
    }

    /**
     * Calculates the relative translation of a module given its corner index.
     * @param cornerIndex The index of the corner
     * @param chassis The chassis configuration
     * @return The relative translation of the module
     */
    public static Translation2d calculateRelativeTranslation(int cornerIndex, SwerveConfig.Chassis chassis) {
        return new Translation2d(
            (cornerIndex >= 2 ? -1 : 1) * chassis.wheelBase().in(Meters) / 2,
            (cornerIndex % 2 == 1 ? -1 : 1) * chassis.trackWidth().in(Meters) / 2
        );
    }
}
