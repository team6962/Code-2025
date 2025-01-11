package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;

// Manipulator is for the coral
public class Manipulator extends SubsystemBase {
    private SparkMax ManipulatorMotor;

    public static enum State {
        IN,
        OUT,
        OFF
    }
    public State state;

    public Command setState(State state) {
        return runEnd(
            () -> this.state = state,
            () -> this.state = State.OFF
        );
    }

    @Override
    public void periodic() {
        if (!ENABLED_SYSTEMS.ENABLE_MANIPULATOR){ 
            return;
        }

        if (RobotState.isDisabled()) {
            state = State.OFF;
        }

        double motorPower = 0.0;

        switch(state) {
            case OFF:
                motorPower = 0.0;
                break;
            case IN:
                motorPower = Preferences.MANIPULATOR.MANIPUALTOR_IN_SPEED;
                break;
            case OUT:
                motorPower = -Preferences.MANIPULATOR.MANIPUALTOR_OUT_SPEED; // Positive and negative might need to be switched for IN and OUT
                break;
        }
        
        ManipulatorMotor.set(motorPower);

        if (RobotContainer.getVoltage() < Preferences.VOLTAGE_LADDER.MANIPULATOR) {
            ManipulatorMotor.stopMotor();
        }
    }

    
}
