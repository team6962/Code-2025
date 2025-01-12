package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.SwerveConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * A simulated swerve module, using the same code as the real one, plus some
 * simulation code.
 */
public class SimulatedModule extends SwerveModule {
    private FlywheelSim driveFlywheel;

    private DCMotorSim steerMotorSim;

    private Angle drivePosition = Rotations.of(0);

    /**
     * The current position of the steer motor in mechanism rotations.
     */
    private Angle steerAngle = Rotations.of(0);

    @Override
    public void configureModule(SwerveConfig constants, Corner corner) {
        super.configureModule(constants, corner);

        driveFlywheel = constants.createDriveMotorSimulation();
        steerMotorSim = constants.createSteerMotorSimulation();
    }

    @Override
    public void periodic() {
        super.periodic();

        Voltage busVoltage = Volts.of(12.0); // TODO: Use BatterySim to estimate bus voltage

        TalonFXSimState driveSim = getDriveMotor().getSimState();

        driveFlywheel.setInputVoltage(MathUtil.clamp(-12.0, driveSim.getMotorVoltage(), 12.0));
        driveFlywheel.update(0.02);

        drivePosition = drivePosition.plus(driveFlywheel.getAngularVelocity().times(Seconds.of(0.02)));

        driveSim.setRawRotorPosition(drivePosition);
        driveSim.setRotorVelocity(driveFlywheel.getAngularVelocity());
        driveSim.setSupplyVoltage(busVoltage);

        TalonFXSimState steerSim = getSteerMotor().getSimState();

        System.out.println(steerSim.getMotorVoltage());

        steerMotorSim.setInputVoltage(MathUtil.clamp(-12.0, steerSim.getMotorVoltage(), 12.0));
        steerMotorSim.update(0.02);

        AngularVelocity steerVelocity = steerMotorSim.getAngularVelocity();

        steerAngle = steerAngle.plus(steerVelocity.times(Seconds.of(0.02)));

        steerSim.setRawRotorPosition(steerAngle.times(getDrivetrainConstants().gearing().steer));
        steerSim.setRotorVelocity(steerVelocity.times(getDrivetrainConstants().gearing().steer));
        steerSim.setSupplyVoltage(busVoltage);

        CANcoderSimState steerEncoderSim = getSteerEncoder().getSimState();

        steerEncoderSim.setRawPosition(steerAngle);
        steerEncoderSim.setVelocity(steerVelocity);
        steerEncoderSim.setSupplyVoltage(busVoltage);
    }
}
