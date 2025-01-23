package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.team6962.lib.swerve.module.SwerveModule.Corner;

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
}
