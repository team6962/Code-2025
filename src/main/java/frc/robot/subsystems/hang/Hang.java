package frc.robot.subsystems.hang;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.HANG;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.SparkMaxUtil;

public class Hang extends SubsystemBase {
    private State state = State.OFF;
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private DutyCycleEncoder hangEncoder;


    public Hang() {
        motor = new SparkMax(CAN.HANG, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
        SparkMaxUtil.saveAndLog(this, motor, motorConfig);
        hangEncoder = new DutyCycleEncoder(DIO.HANG_ENCODER);
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

        double motorPower = 0.0; 

        switch (state) {
            case REVERSE:
                motorPower = -Preferences.HANG.REVERSE_POWER;
                break;
            case CLIMB:
                motorPower = Preferences.HANG.CLIMB_POWER;
                break;
            case OFF:
                motorPower = 0;
                break;
        }

        if ((hangEncoder.get() >= HANG.EXTEND_HEIGHT && motorPower > 0.0) || (hangEncoder.get() <= HANG.RETRACT_HEIGHT && motorPower < 0.0)) {
            motorPower = 0.0;
        }
    
        // Makes sure we dont overshoot our limits for left hang arm

        motor.set(motorPower);

    }
    
    @Override
    public void simulationPeriodic() {
        
    }
}
