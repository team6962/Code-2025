package frc.robot.subsystems.hang;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.SparkMaxUtil;

public class Hang extends SubsystemBase {
    private State state = State.OFF;
    private SparkMax motor;
    private SparkMaxConfig motorConfig;


    public Hang() {
        motor = new SparkMax(CAN.INTAKE, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
        SparkMaxUtil.saveAndLog(this, motor, motorConfig);
    }

    public enum State {
        OFF,
        CLIMB,
        REVERSE
    }

    public Command setPivotState(State state) {
        return runEnd(
            () -> this.state = state,
            () -> this.state = State.OFF
        );
    }

    @Override
    public void periodic() {
        if (!ENABLED_SYSTEMS.ENABLE_HANG) {
            return;
        }

        if (RobotState.isDisabled()) {
            state = State.OFF;
        }

        switch (state) {
            case REVERSE:
                motor.set(-Preferences.HANG.REVERSE_POWER);
                break;
            case CLIMB:
                motor.set(Preferences.HANG.CLIMB_POWER);
                break;
            case OFF:
                motor.set(0);
                break;
        }
    }

    @Override
    public void simulationPeriodic() {
        
    }
}
