package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
        // configurePID
        
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
    public void configurePID(){
        // 
    }

    //getvelocity
    //run with a pid setpoint and a feedforward voltage
}
