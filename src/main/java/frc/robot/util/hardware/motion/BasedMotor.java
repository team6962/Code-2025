package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.team6962.lib.telemetry.Logger;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.hardware.SparkMaxUtil;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasedMotor extends SubsystemBase {
    private BasedElevator.MotorConfig motorConfig;
    private SparkMax motor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController PID;

    public BasedMotor(BasedElevator.Config elevatorConfig, BasedElevator.MotorConfig motorConfig){
        this.motorConfig = motorConfig;
        motor = new SparkMax(motorConfig.canId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        PID = motor.getClosedLoopController();

        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(sparkConfig, motorConfig.inverted, IdleMode.kBrake, (int) elevatorConfig.freeCurrentLimit.in(Amps), (int) elevatorConfig.stallCurrentLimit.in(Amps));
        SparkMaxUtil.configureEncoder(sparkConfig, elevatorConfig.sprocketRadius.in(Meters) / elevatorConfig.gearReduction);
        SparkMaxUtil.configurePID(
            sparkConfig, elevatorConfig.kP, elevatorConfig.kI, elevatorConfig.kD, elevatorConfig.minHeight.in(Meters),elevatorConfig.maxHeight.in(Meters), false);
        SparkMaxUtil.saveAndLog(this, motor, sparkConfig);
    }   

    public void setPosition(Distance Height) {
        encoder.setPosition(Height.in(Meters));
    }

    public Distance getPosition(){
        return Meters.of(encoder.getPosition());
    }

    public LinearVelocity getVelocity(){
        return MetersPerSecond.of(encoder.getVelocity());
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    public double getDutyCycle(){
        return motor.getAppliedOutput();
    }

    public Current getOutputCurrent() {
        return Amps.of(motor.getOutputCurrent());
    }

    public void run(Distance pidSetpoint, Voltage feedforwardVoltage) {
        PID.setReference(pidSetpoint.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVoltage.in(Volts));
    }

    @Override
    public String getName() {
        return motorConfig.name;
    }

    public void logUnder(String path) {
        Logger.logMeasure(path + "/" + getName() + "Motor/position", this::getPosition);
        Logger.logMeasure(path + "/" + getName() + "Motor/velocity", this::getVelocity);
        Logger.logNumber(path + "/" + getName() + "Motor/dutyCycle", this::getDutyCycle);
        Logger.logMeasure(path + "/" + getName() + "Motor/outputCurrent", this::getOutputCurrent);
        Logger.logNumber(path + "/" + getName() + "Motor/busVoltage", motor::getBusVoltage);
        Logger.logNumber(path + "/" + getName() + "Motor/outputVoltage", () -> getDutyCycle() * motor.getBusVoltage());
    }
}
