package com.team6962.lib.test;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Wheel;
import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class DriveModuleTest extends SubsystemBase {
  private TalonFX motor;
  // DCMotorSim is working
  private DCMotorSim motorSim;
  private SwerveConfig swerveConfig = Constants.SWERVE.CONFIG;

  public DriveModuleTest() {
    motor = new TalonFX(39);

    TalonFXConfigurator config = motor.getConfigurator();

    System.out.println(swerveConfig.driveMotor().gains().kS);

    config.apply(
        new Slot0Configs()
            .withKP(0.01)
            .withKD(0.01)
            .withKI(0.1)
            .withKS(swerveConfig.driveMotor().gains().kS)
            .withKV(0.125)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

    config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                Wheel.COLSON.driveMomentOfInertia().in(KilogramSquareMeters),
                Gearing.MK4I_L2_PLUS.drive()),
            DCMotor.getKrakenX60Foc(1));

    setName("DriveModuleTest");
  }

  @Override
  public void periodic() {
    AngularVelocity targetVelocity = swerveConfig.driveMotorMechanismToRotor(FeetPerSecond.of(2));

    Logger.log(getName() + "/targetVelocity", targetVelocity.div(Gearing.MK4I_L2_PLUS.drive()));
    Logger.log(getName() + "/wheelVelocityBefore", motorSim.getAngularVelocity());

    motor.setControl(new VelocityVoltage(targetVelocity));

    TalonFXSimState simState = motor.getSimState();

    motorSim.setInputVoltage(simState.getMotorVoltage());
    motorSim.update(0.02);

    simState.setSupplyVoltage(Volts.of(12));

    simState.setRawRotorPosition(motorSim.getAngularPosition().times(Gearing.MK4I_L2_PLUS.drive()));
    simState.setRotorVelocity(motorSim.getAngularVelocity().times(Gearing.MK4I_L2_PLUS.drive()));
    simState.setRotorAcceleration(
        motorSim.getAngularAcceleration().times(Gearing.MK4I_L2_PLUS.drive()));

    Logger.log(getName() + "/busVoltage", 12.0);
    Logger.log(getName() + "/outputVoltage", simState.getMotorVoltage());
    Logger.log(getName() + "/wheelPosition", motorSim.getAngularPosition());
    Logger.log(getName() + "/wheelVelocity", motorSim.getAngularVelocity());
    Logger.log(getName() + "/wheelAcceleration", motorSim.getAngularAcceleration());

    Logger.log(getName() + "/linearVelocity", motorSim.getAngularVelocity());
  }
}
