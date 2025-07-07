package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
        sparkConfig.closedLoop.minOutput((-12. + elevatorConfig.kG) / 12.);
        SparkMaxUtil.configure(sparkConfig, true, IdleMode.kBrake, 80, 60);
        SparkMaxUtil.configureEncoder(sparkConfig, elevatorConfig.sprocketRadius.in(Meters) / elevatorConfig.gearReduction); // replace with conversion factor
        SparkMaxUtil.configurePID(
            sparkConfig, elevatorConfig.kP, elevatorConfig.kI, elevatorConfig.kD, elevatorConfig.kV, elevatorConfig.minHeight.in(Meters),elevatorConfig.maxHeight.in(Meters), false);
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

    public void run(double pidSetpoint, Voltage feedforwardVoltage) {
        // PID.setReference(pidSetpoint, feedforwardVoltage);
    }
    //run with a pid setpoint and a feedforward voltage
}
