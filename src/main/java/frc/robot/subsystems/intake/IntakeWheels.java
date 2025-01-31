package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.SparkMaxUtil;

public class IntakeWheels extends SubsystemBase {
    private SparkMax motor;
    private SparkMaxConfig motorConfig;

    public IntakeWheels() {
        motor = new SparkMax(CAN.INTAKE_WHEELS, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
        SparkMaxUtil.saveAndLog(this, motor, motorConfig);

        setDefaultCommand(stop());
    }

    public Command setSpeed(DoubleSupplier speed) {
        // TODO: Check for REVLibErrors
        return Commands.run(() -> motor.set(speed.getAsDouble()), this);
    }

    public Command intake() {
        return setSpeed(() -> Preferences.INTAKE.IN_POWER);
    }

    public Command drop() {
        return setSpeed(() -> Preferences.INTAKE.OUT_POWER);
    }

    public Command stop() {
        return setSpeed(() -> 0);
    }

    @Override
    public void periodic() {
        if (!ENABLED_SYSTEMS.ENABLE_INTAKE) {
            motor.disable();
        }
    }
}
