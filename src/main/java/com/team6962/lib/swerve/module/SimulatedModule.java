package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * A simulated swerve module, using the same code as the real one, plus some
 * simulation code.
 */
public class SimulatedModule extends SwerveModule {
    private DriveSim driveSim;
    private SteerSim steerSim;

    private Timer deltaTimer = new Timer();

    @Override
    public void configureModule(SwerveConfig constants, Corner corner) {
        super.configureModule(constants, corner);

        driveSim = new DriveSim(() -> Volts.of(12.0));
        steerSim = new SteerSim(() -> Volts.of(12.0));

        System.out.println("Configured simulated module " + corner);
    }

    @Override
    public void periodic() {
        deltaTimer.stop();

        super.periodic();

        Logger.log(getName() + "/deltaTime", deltaTimer.get());
        Logger.log(getName() + "/busVoltage", 12.0);

        driveSim.update();
        steerSim.update();

        deltaTimer.reset();
        deltaTimer.start();
    }

    private class SteerSim {
        private DCMotorSim steerMotorSim;

        private Angle wheelAngle = Rotations.of(0);

        private Supplier<Voltage> busVoltageSupplier;

        public SteerSim(Supplier<Voltage> busVoltageSupplier) {
            this.busVoltageSupplier = busVoltageSupplier;

            SwerveConfig config = getDrivetrainConstants();

            steerMotorSim = config.createSteerMotorSimulation();
        }

        public void update() {
            Time delta = Seconds.of(deltaTimer.get());

            TalonFXSimState steerSimState = getSteerMotor().getSimState();

            steerSimState.setSupplyVoltage(busVoltageSupplier.get());

            Logger.log(getName() + "/steerInputVoltage", steerSimState.getMotorVoltage());

            steerMotorSim.setInputVoltage(steerSimState.getMotorVoltage());
            steerMotorSim.update(delta.in(Seconds));

            wheelAngle = steerMotorSim.getAngularPosition();

            Logger.log(getName() + "/steerWheelAngle", wheelAngle);

            CTREUtils.check(steerSimState.setRawRotorPosition(wheelAngle.times(getDrivetrainConstants().gearing().steer())));

            CANcoderSimState encoderSimState = getSteerEncoder().getSimState();

            CTREUtils.check(encoderSimState.setRawPosition(wheelAngle));
        }
    }

    private class DriveSim {
        private DCMotorSim driveMotorSim;

        private Angle wheelAngle = Rotations.of(0);

        private Supplier<Voltage> busVoltageSupplier;

        public DriveSim(Supplier<Voltage> busVoltageSupplier) {
            this.busVoltageSupplier = busVoltageSupplier;
            
            SwerveConfig config = getDrivetrainConstants();

            driveMotorSim = config.createDriveMotorSimulation();
        }

        public void update() {
            Time delta = Seconds.of(deltaTimer.get());

            TalonFXSimState driveSimState = getDriveMotor().getSimState();

            Logger.log(getName() + "/driveInputVoltage", driveSimState.getMotorVoltage());

            driveSimState.setSupplyVoltage(busVoltageSupplier.get());

            driveMotorSim.setInputVoltage(driveSimState.getMotorVoltage());
            driveMotorSim.update(delta.in(Seconds));

            wheelAngle = driveMotorSim.getAngularPosition();

            Logger.log(getName() + "/driveWheelAngle", driveMotorSim.getAngularPosition());

            CTREUtils.check(driveSimState.setRawRotorPosition(wheelAngle.times(getDrivetrainConstants().gearing().drive())));
        }
    }
}
