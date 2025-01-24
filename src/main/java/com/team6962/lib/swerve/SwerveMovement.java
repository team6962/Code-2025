package com.team6962.lib.swerve;

import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotContainer;

/**
 * Represents a movement that a swerve drive can take over a single control loop
 * cycle.
 * <p>
 * Movements can be created directly with
 * {@link #setStates(SwerveModuleState[])}
 * or {@link #setSpeeds(ChassisSpeeds)}, or by setting the translation
 * and rotation speeds independently with {@link #setTranslation(Translation2d)}
 * and {@link #setRotation(Rotation2d) setRotation()}. The movement can then be
 * converted into an array of {@link SwerveModuleState}s with
 * {@link #getStates()}.
 * To reuse a movement object, call {@link #clear()} to delete it's state.
 * <p>
 * {@code SwerveMovement} was initially implemented as an interface, but was
 * changed to a class for optimization purposes. It's internals should be hidden,
 * in case it needs to be changed back to an interface.
 */
public class SwerveMovement {
    private SwerveModuleState[] states;
    private ChassisSpeeds speeds;
    private SwerveDriveKinematics kinematics;

    /**
     * Creates a new SwerveMovement object.
     * 
     * @param kinematics The SwerveDriveKinematics object that will be used to
     *                   convert {@link ChassisSpeeds} into
     *                   {@link SwerveModuleState}s
     */
    public SwerveMovement(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    /**
     * Sets the swerve module states of the movement, overriding any previous
     * states or speeds.
     * 
     * @param states The swerve module states, directions and speeds of individual
     *               modules.
     */
    public void setStates(SwerveModuleState[] states) {
        this.states = states;
        speeds = null;
    }

    /**
     * Sets the chassis speeds of the movement, overriding any previous states
     * or speeds.
     * 
     * @param speeds The chassis speeds
     */
    public void setSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
        states = null;
    }

    /**
     * Sets the translation speed of the movement, overriding any previous
     * translation or rotation speeds.
     * 
     * @param translation The translation speed
     */
    public void setTranslation(Translation2d translation) {
        speeds = KinematicsUtils.setTranslationSpeed(speeds, translation);
    }

    /**
     * Sets the rotation speed of the movement, overriding any previous
     * translation or rotation speeds.
     * 
     * @param rotation The rotation speed
     */
    public void setRotation(Rotation2d rotation) {
        speeds = KinematicsUtils.setRotationSpeed(speeds, rotation);
    }

    /**
     * Converts the movement into an array of swerve module states.
     * 
     * @return The swerve module states of the movement.
     */
    public SwerveModuleState[] getStates() {
        if (speeds != null) {
            states = kinematics.toSwerveModuleStates(speeds);

            Logger.log("SwerveMovement/speedsd", RobotContainer.getInstance().swerveDrive.robotToAllianceSpeeds(kinematics.toChassisSpeeds(states)));
        }

        return states;
    }

    /**
     * Clears the movement, deleting any states or speeds that were set.
     */
    public void clear() {
        states = null;
        speeds = null;
    }
}
