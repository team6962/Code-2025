package com.team6962.lib.test;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Wheel;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveModuleTest extends SubsystemBase {
    private TalonFX motor;
    private DCMotorSim motorSim;

    public DriveModuleTest() {
        motor = new TalonFX(39);

        TalonFXConfigurator config = motor.getConfigurator();

        config.apply(new Slot0Configs()
                .withKP(1000));

        config.apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake));

        motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                Wheel.COLSON.driveMomentOfInertia().in(KilogramSquareMeters),
                Gearing.MK4I_L2_PLUS.drive()
            ),
            DCMotor.getKrakenX60Foc(1)
        );

        setName("DriveModuleTest");
    }

    public static AngularVelocity driveMotorMechanismToRotor(LinearVelocity movement, Wheel wheel, Gearing gearing) {
        return RadiansPerSecond.of(movement.in(MetersPerSecond) / wheel.radius().in(Meters) * gearing.drive);
    }

    @Override
    public void periodic() {
        motor.setControl(new VelocityTorqueCurrentFOC(
            driveMotorMechanismToRotor(FeetPerSecond.of(17), Wheel.COLSON, Gearing.MK4I_L2_PLUS)
        ));

        TalonFXSimState simState = motor.getSimState();

        motorSim.setInputVoltage(simState.getMotorVoltage());
        motorSim.update(0.02);

        simState.setSupplyVoltage(Volts.of(12));

        simState.setRawRotorPosition(motorSim.getAngularPosition().times(Gearing.MK4I_L2_PLUS.drive()));
        simState.setRotorVelocity(motorSim.getAngularVelocity().times(Gearing.MK4I_L2_PLUS.drive()));
        simState.setRotorAcceleration(motorSim.getAngularAcceleration().times(Gearing.MK4I_L2_PLUS.drive()));

        Logger.log(getName() + "/busVoltage", 12.0);
        Logger.log(getName() + "/outputVoltage", simState.getMotorVoltage());
        Logger.log(getName() + "/rotorPosition", motorSim.getAngularPosition());
        Logger.log(getName() + "/rotorVelocity", motorSim.getAngularVelocity());
        Logger.log(getName() + "/rotorAcceleration", motorSim.getAngularAcceleration());
    }
}
