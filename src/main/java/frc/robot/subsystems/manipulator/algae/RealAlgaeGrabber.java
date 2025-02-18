package frc.robot.subsystems.manipulator.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealAlgaeGrabber extends AlgaeGrabber {
    /**
     * Represents the motors that control the grabber.
     */
    private Motors motors;

    /**
     * Represents the debouncer that detects if the motors are stalled.
     */
    private Debouncer detectedDebouncer = new Debouncer(1.0, DebounceType.kFalling);
    private boolean detected = false;
    private Timer gripCheckTimer = new Timer();

    public RealAlgaeGrabber() {
        motors = new MotorGroup(
            new NEO550Motor("Algae Grabber/Left Motor", CAN.MANIPULATOR_ALGAE_LEFT, true),
            new NEO550Motor("Algae Grabber/Right Motor", CAN.MANIPULATOR_ALGAE_RIGHT, false)
        );

        Logger.logBoolean(getName() + "/motorsStalled", () -> detected);
        Logger.logNumber(getName() + "/gripCheckTime", () -> gripCheckTimer.get());

        setDefaultCommand(hold());
    }

    /**
     * Represents a motor or group of motors that can be controlled together.
     */
    private static interface Motors {
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

        /**
         * Returns the output velocity of the motor. If there are multiple motors,
         * returns the average velocity.
         * @return the output velocity
         */
        public AngularVelocity getEncoderVelocity();

        public double getOutputDutyCycle();
    }

    /**
     * Represents a group of motors that can be controlled together.
     */
    private static class MotorGroup implements Motors {
        private Motors[] motors;

        public MotorGroup(Motors... motors) {
            this.motors = motors;
        }

        /**
         * Sets the duty cycle of all motors.
         */
        @Override
        public void setDutyCycle(double dutyCycle) {
            for (Motors motor : motors) {
                motor.setDutyCycle(dutyCycle);
            }
        }

        /**
         * Returns the average output current of all motors.
         */
        @Override
        public Current getOutputCurrent() {
            double current = 0;

            for (Motors motor : motors) {
                current += motor.getOutputCurrent().in(Amps);
            }

            return Amps.of(current).div(motors.length);
        }

        @Override
        public AngularVelocity getEncoderVelocity() {
            AngularVelocity w = RPM.of(0);

            for (Motors motor : motors) {
                w = w.plus(motor.getEncoderVelocity());
            }

            return w.div(motors.length);
        }

        @Override
        public double getOutputDutyCycle() {
            double output = 0;

            for (Motors motor : motors) {
                output += motor.getOutputDutyCycle();
            }

            return output / motors.length;
        }
    }

    /**
     * Represents a Spark MAX controlled NEO 550 motor.
     */
    private static class NEO550Motor implements Motors {
        private SparkMax motor;

        public NEO550Motor(String name, int canId, boolean inverted) {
            motor = new SparkMax(canId, MotorType.kBrushless);

            SparkMaxConfig config = new SparkMaxConfig();
            SparkMaxUtil.configure550(config, inverted, IdleMode.kBrake);
            SparkMaxUtil.saveAndLog(name, motor, config);
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

        @Override
        public AngularVelocity getEncoderVelocity() {
            return RPM.of(motor.getEncoder().getVelocity());
        }

        @Override
        public double getOutputDutyCycle() {
            return motor.getAppliedOutput();
        }
    }

    public Command checkGrip() {
        return Commands.parallel(
            runSpeed(MANIPULATOR.ALGAE_GRIP_CHECK_SPEED),
            Commands.race(
                Commands.waitUntil(() -> detected).andThen(() -> expectGamePiece(true)),
                Commands.waitSeconds(MANIPULATOR.ALGAE_GRIP_CHECK_TIME.in(Seconds)).andThen(() -> expectGamePiece(false))
            )
        );
    }

    private Command hold() {
        return run(() -> {
            if (!ENABLED_SYSTEMS.MANIPULATOR || !hasGamePiece()) {
                stopMotors();
            } else if ((hasGamePiece() || detected) && MANIPULATOR.ALGAE_GRIP_CHECK_ENABLED &&
                gripCheckTimer.advanceIfElapsed(MANIPULATOR.ALGAE_GRIP_CHECK_RATE.in(Seconds))) {
                
                checkGrip().schedule();
            } else {
                motors.setDutyCycle(MANIPULATOR.ALGAE_HOLD_SPEED);
            }
        });
    }

    public Command runSpeed(double speed) {
        return run(() -> {
            if (!ENABLED_SYSTEMS.MANIPULATOR) {
                stopMotors();
                return;
            }
            
            motors.setDutyCycle(speed);
        });
    }

    public Command intake() {
        return runSpeed(MANIPULATOR.ALGAE_IN_SPEED).until(() -> detected).andThen(() -> expectGamePiece(true));
    }

    public Command drop() {
        return runSpeed(-MANIPULATOR.ALGAE_OUT_SPEED).until(() -> !detected).andThen(() -> { expectGamePiece(false); stopMotors(); });
    }

    public Command magicButton() {
        return defer(() -> hasGamePiece() ? drop() : intake());
    }

    public Command stop() {
        return run(this::stopMotors);
    }

    public void stopMotors() {
        motors.setDutyCycle(0);
    }

    @Override
    public void periodic() {
        detected = detectedDebouncer.calculate(motors.getEncoderVelocity().abs(RotationsPerSecond) < 1.0 && Math.abs(motors.getOutputDutyCycle()) > 0.025);
    }
}
