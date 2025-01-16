package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A simulated swerve module, using the same code as the real one, plus some
 * simulation code.
 */
public class SimulatedModule extends SwerveModule {
    private Timer deltaTimer = new Timer();
    private Time delta;

    @Override
    public void configureModule(SwerveConfig constants, Corner corner) {
        super.configureModule(constants, corner);

        new DriveSim();
        new SteerSim();
    }

    @Override
    public void periodic() {
        deltaTimer.stop();

        delta = Seconds.of(deltaTimer.get());

        deltaTimer.reset();
        deltaTimer.start();

        super.periodic();
    }

    private class SteerSim extends SubsystemBase {
        private DCMotorSim steerMotorSim;

        public SteerSim() {
            steerMotorSim = getDrivetrainConstants().createSteerMotorSimulation();
        }

        @Override
        public void periodic() {
            TalonFXSimState steerSimState = getSteerMotor().getSimState();

            steerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            Logger.log(getName() + "/steerSimInVolts", steerSimState.getMotorVoltage());

            steerMotorSim.setInputVoltage(steerSimState.getMotorVoltage());
            steerMotorSim.update(delta.in(Seconds));

            CTREUtils.check(steerSimState.setRawRotorPosition(steerMotorSim.getAngularPosition().times(getDrivetrainConstants().gearing().steer())));

            CANcoderSimState encoderSimState = getSteerEncoder().getSimState();

            CTREUtils.check(encoderSimState.setRawPosition(steerMotorSim.getAngularPosition()));
        }
    }

    private class DriveSim extends SubsystemBase {
        private DCMotorSim driveMotorSim;

        public DriveSim() {
            driveMotorSim = getDrivetrainConstants().createDriveMotorSimulation();
        }

        @Override
        public void periodic() {
            TalonFXSimState driveSimState = getDriveMotor().getSimState();

            Logger.log(getName() + "/driveSimInputVoltage", driveSimState.getMotorVoltage());

            driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            driveMotorSim.setInputVoltage(driveSimState.getMotorVoltage());
            driveMotorSim.update(delta.in(Seconds));

            double gearing = getDrivetrainConstants().gearing().drive();

            CTREUtils.check(driveSimState.setRawRotorPosition(driveMotorSim.getAngularPosition().times(gearing)));
            CTREUtils.check(driveSimState.setRotorVelocity(driveMotorSim.getAngularVelocity().times(gearing)));
            CTREUtils.check(driveSimState.setRotorAcceleration(driveMotorSim.getAngularAcceleration().times(gearing)));
        }
    }
}
