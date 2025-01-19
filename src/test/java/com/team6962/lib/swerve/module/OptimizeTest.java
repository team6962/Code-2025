package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

public class OptimizeTest {
    private void testOptimization(SwerveModuleState state, Angle start, SwerveModuleState expectedOutput) {
        SwerveModuleState optimized = SimulatedModule.optimizeStateForTalon(state, start);

        assertEquals(expectedOutput.angle.getRotations(), optimized.angle.getRotations(), 0.0001);
        assertEquals(expectedOutput.speedMetersPerSecond, optimized.speedMetersPerSecond, 0.0001);
    }

    @Test
    public void optimizeTestNoMovement() {
        testOptimization(
            new SwerveModuleState(
                1,
                Rotation2d.fromDegrees(0)
            ),
            Degrees.of(0),
            new SwerveModuleState(
                1,
                Rotation2d.fromDegrees(0)
            )
        );
    }

    @Test
    public void optimizeTestInvertSpeed() {
        testOptimization(
            new SwerveModuleState(
                1,
                Rotation2d.fromDegrees(0)
            ),
            Degrees.of(120),
            new SwerveModuleState(
                -1,
                Rotation2d.fromDegrees(180)
            )
        );
    }

    @Test
    public void optimizeTestInvertExtraTarget() {
        testOptimization(
            new SwerveModuleState(
                1,
                Rotation2d.fromDegrees(360)
            ),
            Degrees.of(120),
            new SwerveModuleState(
                -1,
                Rotation2d.fromDegrees(180)
            )
        );
    }

    @Test
    public void optimizeTestExtraMeasure() {
        testOptimization(
            new SwerveModuleState(
                1,
                Rotation2d.fromDegrees(360)
            ),
            Degrees.of(720 - 10),
            new SwerveModuleState(
                1,
                Rotation2d.fromDegrees(720)
            )
        );
    }

    @Test
    public void optimizeTestInvertExtraMeasure() {
        testOptimization(
            new SwerveModuleState(
                1,
                Rotation2d.fromDegrees(360)
            ),
            Degrees.of(720 + 120),
            new SwerveModuleState(
                -1,
                Rotation2d.fromDegrees(720 + 180)
            )
        );
    }

    @Test
    public void optimizeTestHigherSpeed() {
        testOptimization(
            new SwerveModuleState(
                14.3,
                Rotation2d.fromDegrees(360)
            ),
            Degrees.of(720 - 10),
            new SwerveModuleState(
                14.3,
                Rotation2d.fromDegrees(720)
            )
        );
    }
}
