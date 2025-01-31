package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

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

public class IntakePivot extends SubsystemBase {
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;

    public IntakePivot() {
        motor = new SparkMax(CAN.INTAKE, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
        SparkMaxUtil.configurePID(motorConfig, 1.0, 0.0, 0.0, 0.0, false);
        SparkMaxUtil.saveAndLog(this, motor, motorConfig);

        closedLoopController = motor.getClosedLoopController();

        setDefaultCommand(raise().repeatedly());
    }

    public Command setAngle(Supplier<Angle> angle) {
         // TODO: Check for REVLibErrors
        return Commands.run(() -> closedLoopController.setReference(angle.get().in(Rotations), ControlType.kPosition))
            .until(() -> motor.get() == angle.get().in(Rotations));
    }

    public Command lower() {
        return setAngle(() -> Preferences.INTAKE.PIVOT_DOWN);
    }

    public Command raise() {
        return setAngle(() -> Preferences.INTAKE.PIVOT_UP);
    }

    @Override
    public void periodic() {
        if (!ENABLED_SYSTEMS.ENABLE_INTAKE) {
            motor.disable();
        }
    }
}
