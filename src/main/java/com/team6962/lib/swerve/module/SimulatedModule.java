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

/**
 * A simulated swerve module, using the same code as the real one, plus some
 * simulation code.
 */
public class SimulatedModule extends SwerveModule {
    private Timer deltaTimer = new Timer();
    private Time delta;

    private DriveSim driveSim;
    private SteerSim steerSim;

    @Override
    public void configureModule(SwerveConfig constants, Corner corner) {
        super.configureModule(constants, corner);

        driveSim = new DriveSim();
        steerSim = new SteerSim();
    }

    @Override
    public void periodic() {
        deltaTimer.stop();

        delta = Seconds.of(deltaTimer.get());

        deltaTimer.reset();
        deltaTimer.start();

        super.periodic();

        driveSim.update();
        steerSim.update();
    }

    private class SteerSim {
        private DCMotorSim steerMotorSim;

        public SteerSim() {
            steerMotorSim = getDrivetrainConstants().createSteerMotorSimulation();
        }

        public void update() {
            TalonFXSimState steerSimState = getSteerMotor().getSimState();

            steerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            Logger.log(getName() + "/steerSimInVolts", -steerSimState.getMotorVoltage());

            steerMotorSim.setInputVoltage(-steerSimState.getMotorVoltage());
            steerMotorSim.update(delta.in(Seconds));

            CTREUtils.check(steerSimState.setRawRotorPosition(steerMotorSim.getAngularPosition().times(getDrivetrainConstants().gearing().steer())));

            CANcoderSimState encoderSimState = getSteerEncoder().getSimState();

            CTREUtils.check(encoderSimState.setRawPosition(steerMotorSim.getAngularPosition()));
        }
    }

    private class DriveSim {
        private DCMotorSim driveMotorSim;

        public DriveSim() {
            driveMotorSim = getDrivetrainConstants().createDriveMotorSimulation();
        }

        public void update() {
            TalonFXSimState driveSimState = getDriveMotor().getSimState();

            Logger.log(getName() + "/driveSimInputVoltage", driveSimState.getMotorVoltage());

            driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            driveMotorSim.setInputVoltage(driveSimState.getMotorVoltage());
            driveMotorSim.update(delta.in(Seconds));

            double gearing = getDrivetrainConstants().gearing().drive();

            Logger.log(getName() + "/driveSimRotorPosition", driveMotorSim.getAngularPosition().times(gearing));
            Logger.log(getName() + "/driveSimRotorVelocity", driveMotorSim.getAngularVelocity().times(gearing));
            Logger.log(getName() + "/driveSimRotorAcceleration", driveMotorSim.getAngularAcceleration().times(gearing));

            CTREUtils.check(driveSimState.setRawRotorPosition(driveMotorSim.getAngularPosition().times(gearing)));
            CTREUtils.check(driveSimState.setRotorVelocity(driveMotorSim.getAngularVelocity().times(gearing)));
            CTREUtils.check(driveSimState.setRotorAcceleration(driveMotorSim.getAngularAcceleration().times(gearing)));
        }
    }
}
