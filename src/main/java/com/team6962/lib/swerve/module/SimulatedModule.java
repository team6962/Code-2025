package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    }

    @Override
    public void periodic() {
        deltaTimer.stop();

        super.periodic();

        driveSim.periodic();
        steerSim.periodic();

        deltaTimer.reset();
        deltaTimer.start();
    }

    private class SteerSim extends SubsystemBase {
        private FlywheelSim steerMotorSim;

        private Angle wheelAngle = Rotations.of(0);

        private Supplier<Voltage> busVoltageSupplier;

        public SteerSim(Supplier<Voltage> busVoltageSupplier) {
            this.busVoltageSupplier = busVoltageSupplier;

            SwerveConfig config = getDrivetrainConstants();

            steerMotorSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    config.steerMotor().stats(),
                    config.wheel().steerMomentOfInertia().in(KilogramSquareMeters),
                    config.gearing().steer()
                ),
                config.steerMotor().stats()
            );
        }

        @Override
        public void periodic() {
            Time delta = Seconds.of(deltaTimer.get());

            TalonFXSimState steerSimState = getSteerMotor().getSimState();

            steerSimState.setSupplyVoltage(busVoltageSupplier.get());

            steerMotorSim.setInputVoltage(steerSimState.getMotorVoltage());
            steerMotorSim.update(delta.in(Seconds));

            wheelAngle = wheelAngle.plus(steerMotorSim.getAngularVelocity().times(delta));

            CTREUtils.check(steerSimState.setRawRotorPosition(wheelAngle.times(getDrivetrainConstants().gearing().steer())));

            CANcoderSimState encoderSimState = getSteerEncoder().getSimState();

            CTREUtils.check(encoderSimState.setRawPosition(wheelAngle));
        }
    }

    private class DriveSim extends SubsystemBase {
        private FlywheelSim driveMotorSim;

        private Angle wheelAngle = Rotations.of(0);

        private Supplier<Voltage> busVoltageSupplier;

        public DriveSim(Supplier<Voltage> busVoltageSupplier) {
            this.busVoltageSupplier = busVoltageSupplier;
            
            SwerveConfig config = getDrivetrainConstants();

            driveMotorSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    config.driveMotor().stats(),
                    config.wheel().driveMomentOfInertia().in(KilogramSquareMeters),
                    config.gearing().drive()
                ),
                config.driveMotor().stats()
            );
        }

        @Override
        public void periodic() {
            Time delta = Seconds.of(deltaTimer.get());

            TalonFXSimState driveSimState = getDriveMotor().getSimState();

            driveSimState.setSupplyVoltage(busVoltageSupplier.get());

            driveMotorSim.setInputVoltage(driveSimState.getMotorVoltage());
            driveMotorSim.update(delta.in(Seconds));

            wheelAngle = wheelAngle.plus(driveMotorSim.getAngularVelocity().times(delta));

            CTREUtils.check(driveSimState.setRawRotorPosition(wheelAngle.times(getDrivetrainConstants().gearing().drive())));
        }
    }
}
