package com.team6962.lib.test;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Module;
import com.team6962.lib.swerve.SwerveConfig.Motor;
import com.team6962.lib.swerve.SwerveConfig.Wheel;
import com.team6962.lib.swerve.module.SimulatedModule;
import com.team6962.lib.swerve.module.SwerveModule.Corner;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class SwerveModuleTest extends SubsystemBase {
    private SimulatedModule module;

    public SwerveModuleTest() {
        /* create the TalonFX */
        module = new SimulatedModule();

        module.configureModule(Constants.SWERVE.CONFIG, Corner.FRONT_LEFT);
    }

    @Override
    public void periodic() {
        module.driveState(new SwerveModuleState(FeetPerSecond.of(Math.sin(Timer.getFPGATimestamp() * 0.1) * 15).in(MetersPerSecond), Rotation2d.fromRotations(Timer.getFPGATimestamp() * 0.1)));
    }
}
