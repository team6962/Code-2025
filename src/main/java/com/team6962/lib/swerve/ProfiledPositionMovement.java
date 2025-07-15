package com.team6962.lib.swerve;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.swerve.module.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ProfiledPositionMovement implements SwerveMovement {
    private SwerveModulePosition[] targetPositions;
    private LinearVelocity[] maximumVelocities;
    private LinearAcceleration[] maximumAccelerations;

    public ProfiledPositionMovement(
        SwerveModulePosition[] targetPositions,
        LinearVelocity[] maximumVelocities,
        LinearAcceleration[] maximumAccelerations
    ) {
        if (targetPositions.length != 4 || maximumVelocities.length != 4 || maximumAccelerations.length != 4) {
            throw new IllegalArgumentException("Target positions, velocities, and accelerations must have exactly 4 elements.");
        }

        this.targetPositions = targetPositions;
        this.maximumVelocities = maximumVelocities;
        this.maximumAccelerations = maximumAccelerations;
    }

    @Override
    public void execute(SwerveCore drivetrain) {
        for (int i = 0; i < 4; i++) {
            SwerveModule module = drivetrain.getModules()[i];

            module.drive(
                SwerveMovement.dynamicMotionMagicVoltage
                    .withPosition(
                        module.getDrivetrainConstants().driveMotorMechanismToRotor(Meters.of(targetPositions[i].distanceMeters))
                    )
                    .withVelocity(module.getDrivetrainConstants().driveMotorMechanismToRotor(maximumVelocities[i]))
                    .withAcceleration(module.getDrivetrainConstants().driveMotorMechanismToRotor(maximumAccelerations[i])),
                // TODO: Handle reverse wheel rotation
                SwerveMovement.positionVoltage
                    .withPosition(SwerveModule.optimizeSteerAngle(
                        targetPositions[i].angle.getMeasure(),
                        module.getSteerAngle()
                    ))
            );
        }
    }

    @Override
    public SwerveMovement cleared() {
        return null;
    }
}
