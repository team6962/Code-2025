package frc.robot.subsystems.intake;

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

public class Intake extends SubsystemBase {
    private State pivotState = State.OFF;
    private State wheelsState = State.OFF;

    private SparkMax pivotMotor;
    private SparkMax wheelsMotor;

    private SparkMaxConfig pivotMotorConfig;
    private SparkMaxConfig wheelsMotorConfig;



    public Intake() {
        pivotMotor = new SparkMax(CAN.INTAKE, MotorType.kBrushless);
        pivotMotorConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(pivotMotorConfig, false, IdleMode.kBrake);
        SparkMaxUtil.saveAndLog(this, pivotMotor, pivotMotorConfig);

        wheelsMotor = new SparkMax(CAN.INTAKE, MotorType.kBrushless);
        wheelsMotorConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(wheelsMotorConfig, false, IdleMode.kBrake);
        SparkMaxUtil.saveAndLog(this, wheelsMotor, wheelsMotorConfig);
    }

    public enum State {
        OFF,
        IN,
        OUT
    }

    public Command setPivotState(State state) {
        return runEnd(
            () -> this.pivotState = state,
            () -> this.pivotState = State.OFF
        );
    }

    public Command setWheelsState(State state) {
        return runEnd(
            () -> this.wheelsState = state,
            () -> this.wheelsState = State.OFF
        );
    }

    @Override
    public void periodic() {
        if (!ENABLED_SYSTEMS.ENABLE_INTAKE) {
            return;
        }

        if (RobotState.isDisabled()) {
            pivotState = State.OFF;
        }
    }

    @Override
    public void simulationPeriodic() {
        switch (pivotState) {
            case IN:
                pivotMotor.set(-Preferences.INTAKE.IN_POWER);
                break;
            case OUT:
                pivotMotor.set(Preferences.INTAKE.OUT_POWER);
                break;
            case OFF:
                pivotMotor.set(0);
                break;
        }

        switch (wheelsState) {
            case IN:
                wheelsMotor.set(-Preferences.INTAKE.IN_POWER);
                break;
            case OUT:
                wheelsMotor.set(Preferences.INTAKE.OUT_POWER);
                break;
            case OFF:
                wheelsMotor.set(0);
                break;
        }
    }
}
