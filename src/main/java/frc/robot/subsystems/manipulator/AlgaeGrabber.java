package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.swerve.SwerveConfig.Motor;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class AlgaeGrabber extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private boolean holding;
    private Debouncer stallDebouncer;
    private boolean stalled;

    public AlgaeGrabber(
        int leftCanId,
        int rightCanId
    ) {
        leftMotor = new SparkMax(leftCanId, MotorType.kBrushless);
        SparkMaxUtil.configureAndLog550(leftMotor, new SparkMaxConfig(), true, IdleMode.kBrake);
        Logger.logMeasure(getName() + "/LeftMotor/current", () -> Amps.of(leftMotor.getOutputCurrent()));

        rightMotor = new SparkMax(rightCanId, MotorType.kBrushless);
        SparkMaxUtil.configureAndLog550(rightMotor, new SparkMaxConfig(), true, IdleMode.kBrake);
        Logger.logMeasure(getName() + "/RightMotor/current", () -> Amps.of(rightMotor.getOutputCurrent()));

        stallDebouncer = new Debouncer(0.1, DebounceType.kFalling);
    }

    public Command hold() {
        return Commands.run(() -> {
            if (!ENABLED_SYSTEMS.MANIPULATOR || !holding) {
                leftMotor.set(0);
                rightMotor.set(0);
            } else if (stalled) {
                leftMotor.set(0);
                rightMotor.set(0);
            } else {
                leftMotor.set(MANIPULATOR.ALGAE_HOLD_SPEED);
                rightMotor.set(MANIPULATOR.ALGAE_HOLD_SPEED);
            }
        }, this);
    }

    // public Command runMultiple(double speed) {
    //   return Commands.run(() -> {
    //     speed = isEnabled.getAsBoolean() ? speed : 0;
    //   }, this);
    // }

    public Command run(double speed) {
        return Commands.run(() -> {
            leftMotor.set(speed);
            rightMotor.set(speed);
        }, this);
    }

    public Command intake() {
        return run(MANIPULATOR.ALGAE_IN_SPEED).until(() -> stalled).andThen(() -> holding = true);
    }

    public Command drop() {
        return run(MANIPULATOR.ALGAE_OUT_SPEED).alongWith(Commands.waitUntil(() -> !stalled).andThen(() -> holding = false));
    }

    public Command stop() {
        return Commands.run(() -> {
            leftMotor.set(0);
            rightMotor.set(0);
        }, this);
    }

    @Override
    public void periodic() {
        stalled = stallDebouncer.calculate(
            leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent() > MANIPULATOR.ALGAE_DETECT_CURRENT.in(Amps)
        );
    }
}
