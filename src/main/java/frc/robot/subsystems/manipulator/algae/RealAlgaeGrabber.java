package frc.robot.subsystems.manipulator.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealAlgaeGrabber extends AlgaeGrabber {
    private Motor motors;
    private Debouncer stallDebouncer = new Debouncer(0.1, DebounceType.kFalling);
    private boolean stalled = false;
    private Timer gripCheckTimer = new Timer();

    public RealAlgaeGrabber() {
        motors = new CombinedMotor(
            new RealMotor("Algae Grabber/Left Motor", CAN.MANIPULATOR_ALGAE_LEFT, true),
            new RealMotor("Algae Grabber/Right Motor", CAN.MANIPULATOR_ALGAE_RIGHT, false)
        );

        Logger.logBoolean(getName() + "/motorsStalled", () -> stalled);
        Logger.logNumber(getName() + "/gripCheckTime", () -> gripCheckTimer.get());

        setDefaultCommand(hold());
    }

    private static interface Motor {
        /**
         * Sets the duty cycle of the motor.
         * @param dutyCycle the duty cycle to set
         */
        public void setDutyCycle(double dutyCycle);

        /**
         * Returns the output current of the motor. If there are multiple motors,
         * returns the average current.
         * @return the output current
         */
        public Current getOutputCurrent();
    }

    private static class CombinedMotor implements Motor {
        private Motor[] motors;

        public CombinedMotor(Motor... motors) {
            this.motors = motors;
        }

        /**
         * Sets the duty cycle of all motors.
         */
        @Override
        public void setDutyCycle(double dutyCycle) {
            for (Motor motor : motors) {
                motor.setDutyCycle(dutyCycle);
            }
        }

        /**
         * Returns the average output current of all motors.
         */
        @Override
        public Current getOutputCurrent() {
            double current = 0;

            for (Motor motor : motors) {
                current += motor.getOutputCurrent().in(Amps);
            }

            return Amps.of(current).div(motors.length);
        }
    }

    private static class RealMotor implements Motor {
        private SparkMax motor;

        public RealMotor(String name, int canId, boolean inverted) {
            motor = new SparkMax(canId, MotorType.kBrushless);

            SparkMaxConfig config = new SparkMaxConfig();
            SparkMaxUtil.configureAndLog550(motor, config, inverted, IdleMode.kBrake);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            Logger.logMeasure(name + "/outputCurrent", () -> Amps.of(motor.getOutputCurrent()));
        }

        /**
         * Sets the duty cycle of the motor.
         */
        @Override
        public void setDutyCycle(double dutyCycle) {
            motor.set(dutyCycle);
        }

        /**
         * Returns the output current of the motor.
         */
        @Override
        public Current getOutputCurrent() {
            return Amps.of(motor.getOutputCurrent());
        }
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
                motors.setDutyCycle(0);
            } else if (!hasGamePiece()) {
                motors.setDutyCycle(0);

                if (MANIPULATOR.ALGAE_GRIP_CHECK_ENABLED &&
                    gripCheckTimer.advanceIfElapsed(MANIPULATOR.ALGAE_GRIP_CHECK_RATE.in(Seconds))) {
                    
                    checkGrip().schedule();
                }
            } else if (stalled) {
                motors.setDutyCycle(0);
            } else {
                motors.setDutyCycle(MANIPULATOR.ALGAE_HOLD_SPEED);
            }
        }, this);
    }

    public Command run(double speed) {
        return Commands.run(() -> {
            if (!ENABLED_SYSTEMS.MANIPULATOR) {
                motors.setDutyCycle(0);
                return;
            }
            
            motors.setDutyCycle(speed);
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
        return Commands.run(() -> motors.setDutyCycle(0), this);
    }

    @Override
    public void periodic() {
        stalled = stallDebouncer.calculate(motors.getOutputCurrent().gt(MANIPULATOR.ALGAE_DETECT_CURRENT));
    }
}
