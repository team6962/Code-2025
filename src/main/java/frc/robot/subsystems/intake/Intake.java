package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.SparkMaxUtil;

public class Intake extends SubsystemBase {
    private SparkMax pivotMotor;
    private SparkMax wheelsMotor;

    private SparkMaxConfig pivotMotorConfig;
    private SparkMaxConfig wheelsMotorConfig;

    private SparkClosedLoopController pivotMotorController;

    public Intake() {
        pivotMotor = new SparkMax(CAN.INTAKE, MotorType.kBrushless);
        pivotMotorConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(pivotMotorConfig, false, IdleMode.kBrake);
        SparkMaxUtil.configurePID(pivotMotorConfig, 1.0, 0.0, 0.0, 0.0, false);
        SparkMaxUtil.saveAndLog(this, pivotMotor, pivotMotorConfig);

        pivotMotorController = pivotMotor.getClosedLoopController();

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

    public Command setPivotAngle(Supplier<Angle> angle) {
         // TODO: Check for REVLibErrors
        return Commands.run(() -> pivotMotorController.setReference(angle.get().in(Rotations), ControlType.kPosition))
            .until(() -> pivotMotor.get() == angle.get().in(Rotations));
    }

    public Command lowerPivot() {
        return setPivotAngle(() -> Preferences.INTAKE.PIVOT_DOWN);
    }

    public Command raisePivot() {
        return setPivotAngle(() -> Preferences.INTAKE.PIVOT_UP);
    }

    public Command setWheelsSpeed(DoubleSupplier speed) {
        // TODO: Check for REVLibErrors
        return Commands.run(() -> wheelsMotor.set(speed.getAsDouble()));
    }

    public Command setWheelsIntaking() {
        return setWheelsSpeed(() -> Preferences.INTAKE.IN_POWER);
    }

    public Command setWheelsDropping() {
        return setWheelsSpeed(() -> Preferences.INTAKE.OUT_POWER);
    }

    public Command startIntake() {
        return Commands.sequence(
            raisePivot(),
            setWheelsIntaking()
        );
    }

    public Command startDropping() {
        return Commands.sequence(
            lowerPivot(),
            setWheelsDropping()
        );
    }

    public Command stop() {
        return Commands.sequence(
            setWheelsSpeed(() -> 0),
            raisePivot()
        );
    }

    public Command intakeAlgae() {
        return Commands.sequence(
            startIntake(),
            Commands.waitSeconds(1.0),
            stop()
        );
    }

    public Command dropAlgae() {
        return Commands.sequence(
            startDropping(),
            Commands.waitSeconds(1.0),
            stop()
        );
    }

    @Override
    public void periodic() {
        if (!ENABLED_SYSTEMS.ENABLE_INTAKE) {
            pivotMotor.set(0);
            wheelsMotor.set(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        
    }
}
