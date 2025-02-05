package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealIntakePivot extends IntakePivot {
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private Angle targetAngle;

    public RealIntakePivot() {
        motor = new SparkMax(CAN.INTAKE_PIVOT, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
        SparkMaxUtil.configurePID(motorConfig, 1.0, 0.0, 0.0, 0.0, false);
        SparkMaxUtil.saveAndLog(this, motor, motorConfig);

        closedLoopController = motor.getClosedLoopController();

        setDefaultCommand(raise().repeatedly());
    }

    public Command setAngle(Supplier<Angle> angle) {
        // TODO: Check for REVLibErrors
        return Commands.run(
                () -> {
                    targetAngle = angle.get();

                    closedLoopController.setReference(angle.get().in(Rotations), ControlType.kPosition);
                }, this)
            .until(() -> motor.get() == angle.get().in(Rotations));
    }
    
    @Override
    protected Voltage getAppliedVoltage() {
        return Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @Override
    protected Angle getMeasuredAngle() {
        return Rotations.of(motor.getAbsoluteEncoder().getPosition());
    }

    @Override
    protected Angle getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void periodic() {
        if (!ENABLED_SYSTEMS.INTAKE) {
        motor.disable();
        }
    }
}
