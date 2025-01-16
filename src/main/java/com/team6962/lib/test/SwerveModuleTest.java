package com.team6962.lib.test;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Module;
import com.team6962.lib.swerve.SwerveConfig.Motor;
import com.team6962.lib.swerve.SwerveConfig.Wheel;
import com.team6962.lib.swerve.module.SimulatedModule;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.swerve.module.SwerveModule.Corner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModuleTest extends SubsystemBase {
    private SwerveModule module;

    public SwerveModuleTest() {
        module = new SimulatedModule();

        module.configureModule(new SwerveConfig(
            null,
            Gearing.MK4I_L2_PLUS,
            new Module[] {
                new Module(10, 11, 12, Degrees.of(0)),
                new Module(20, 21, 22, Degrees.of(0)),
                new Module(30, 31, 32, Degrees.of(0)),
                new Module(40, 41, 42, Degrees.of(0))
            },
            new Motor(
                DCMotor.getKrakenX60(1),
                new Slot0Configs().withKP(1000),
                Amps.of(300)
            ),
            new Motor(
                DCMotor.getKrakenX60(1),
                new Slot0Configs().withKP(1000),
                Amps.of(300)
            ),
            Wheel.COLSON,
            null
        ), Corner.FRONT_LEFT);
    }

    @Override
    public void periodic() {
        module.driveState(new SwerveModuleState(1, Rotation2d.fromDegrees(0)));
    }
}
