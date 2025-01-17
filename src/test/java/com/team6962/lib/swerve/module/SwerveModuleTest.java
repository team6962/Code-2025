package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Module;
import com.team6962.lib.swerve.SwerveConfig.Motor;
import com.team6962.lib.swerve.SwerveConfig.Wheel;
import com.team6962.lib.swerve.module.SwerveModule.Corner;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class SwerveModuleTest implements AutoCloseable {
    private SimulatedModule module;

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        /* create the TalonFX */
        module = new SimulatedModule();

        module.configureModule(new SwerveConfig(
            null,
            Gearing.MK4I_L2_PLUS,
            new Module[] {
                new Module(13, 11, 12, Degrees.of(0)),
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

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @AfterEach
   void shutdown() throws Exception {
      close();
   }

    @Test
    public void robotIsEnabled() {
        /* verify that the robot is enabled */
        assertTrue(DriverStation.isEnabled());
    }

    @Test
    public void testSteer() {
        for (int i = 0; i < 100; i++) {
            Timer.delay(0.020);

            module.driveState(new SwerveModuleState(0, Rotation2d.fromDegrees(10)));
        }

        SwerveModuleState currentState = module.getState();

        assertEquals(currentState.angle.getDegrees(), 10, 1e-2);
    }

    @Override
    public void close() throws Exception {
        module.close();
    }
}
