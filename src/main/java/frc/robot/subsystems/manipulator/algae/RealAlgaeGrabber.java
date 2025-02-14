package frc.robot.subsystems.manipulator.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealAlgaeGrabber extends AlgaeGrabber {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private Debouncer stallDebouncer = new Debouncer(0.1, DebounceType.kFalling);
    private boolean stalled = false;
    private Timer gripCheckTimer = new Timer();

    public RealAlgaeGrabber() {
        leftMotor = new SparkMax(CAN.MANIPULATOR_ALGAE_LEFT, MotorType.kBrushless);
        SparkMaxUtil.configureAndLog550(leftMotor, new SparkMaxConfig(), true, IdleMode.kBrake);

        Logger.logBoolean(getName() + "/motorsStalled", () -> stalled);
        Logger.logNumber(getName() + "/gripCheckTime", () -> gripCheckTimer.get());
        Logger.logMeasure(getName() + "/LeftMotor/current", () -> Amps.of(leftMotor.getOutputCurrent()));

        rightMotor = new SparkMax(CAN.MANIPULATOR_ALGAE_RIGHT, MotorType.kBrushless);
        SparkMaxUtil.configureAndLog550(rightMotor, new SparkMaxConfig(), true, IdleMode.kBrake);
        Logger.logMeasure(getName() + "/RightMotor/current", () -> Amps.of(rightMotor.getOutputCurrent()));

        setDefaultCommand(hold());
    }

    public Command checkGrip() {
        return Commands.parallel(
            run(MANIPULATOR.ALGAE_IN_SPEED),
            Commands.race(
                Commands.waitUntil(() -> stalled).andThen(() -> expectGamePiece(true)),
                Commands.waitSeconds(MANIPULATOR.ALGAE_GRIP_CHECK_TIME.in(Seconds)).andThen(() -> expectGamePiece(false))
            )
        );
    }

    private Command hold() {
        return Commands.run(() -> {
            if (!ENABLED_SYSTEMS.MANIPULATOR) {
                leftMotor.set(0);
                rightMotor.set(0);
            } else if (!hasGamePiece()) {
                leftMotor.set(0);
                rightMotor.set(0);

                if (MANIPULATOR.ALGAE_GRIP_CHECK_ENABLED &&
                    gripCheckTimer.advanceIfElapsed(MANIPULATOR.ALGAE_GRIP_CHECK_RATE.in(Seconds))) {
                    
                    checkGrip().schedule();

                    gripCheckTimer.reset();
                }
            } else if (stalled) {
                leftMotor.set(0);
                rightMotor.set(0);
            } else {
                leftMotor.set(MANIPULATOR.ALGAE_HOLD_SPEED);
                rightMotor.set(MANIPULATOR.ALGAE_HOLD_SPEED);
            }
        }, this);
    }

    public Command run(double speed) {
        return Commands.run(() -> {
            if (!ENABLED_SYSTEMS.MANIPULATOR) {
                leftMotor.set(0);
                rightMotor.set(0);
                return;
            }
            
            leftMotor.set(speed);
            rightMotor.set(speed);
        }, this);
    }

    public Command intake() {
        return run(MANIPULATOR.ALGAE_IN_SPEED).until(() -> stalled).andThen(() -> expectGamePiece(true));
    }

    public Command drop() {
        return run(MANIPULATOR.ALGAE_OUT_SPEED).alongWith(Commands.waitUntil(() -> !stalled).andThen(() -> expectGamePiece(false)));
    }

    public Command action() {
        return Commands.defer(() -> hasGamePiece() ? drop() : intake(), Set.of(this));
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
