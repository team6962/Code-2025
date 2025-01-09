package com.team6962.lib.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public final class RotationUtils {
    private RotationUtils() {
    }

    /**
     * Returns the shortest distance between two angles
     * @param x The first angle
     * @param y The second angle
     * @return The shortest distance between the two angles, which is always
     * in the range [0, 180˚]
     */
    public static Rotation2d shortestDistance(Rotation2d x, Rotation2d y) {
        return Rotation2d.fromDegrees(Math.abs(Math.IEEEremainder(y.getDegrees() - x.getDegrees(), 360)));
    }

    /**
     * Normalizes an angle to the range [0, 1)
     * @param angle The angle to normalize
     * @return The normalized angle, essentially the angle modulo 360˚
     */
    public static Rotation2d normalize(Rotation2d angle) {
        return Rotation2d.fromRotations(Math.IEEEremainder(angle.getRotations(), 1));
    }
}
