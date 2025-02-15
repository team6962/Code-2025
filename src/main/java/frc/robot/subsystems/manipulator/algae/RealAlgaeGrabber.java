package frc.robot.subsystems.manipulator.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.swerve.SwerveConfig.Motor;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealAlgaeGrabber extends AlgaeGrabber {
    private Motor motors;
    private Debouncer stallDebouncer = new Debouncer(1.0, DebounceType.kFalling);
    private boolean stalled = false;
    private Timer gripCheckTimer = new Timer();

    public RealAlgaeGrabber() {
        motors = new CombinedMotor(
            new RealMotor("Algae Grabber/Left Motor", CAN.MANIPULATOR_ALGAE_LEFT, true),
            new RealMotor("Algae Grabber/Right Motor", CAN.MANIPULATOR_ALGAE_RIGHT, false)
        );

        Logger.logBoolean(getName() + "/motorsStalled", () -> stalled);
        Logger.logNumber(getName() + "/gripCheckTime", () -> gripCheckTimer.get());

        // setDefaultCommand(hold());
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

        /**
         * Returns the output velocity of the motor. If there are multiple motors,
         * returns the average velocity.
         * @return the output velocity
         */
        public AngularVelocity getEncoderVelocity();

        public double getOutputDutyCycle();
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

        @Override
        public AngularVelocity getEncoderVelocity() {
            AngularVelocity w = RPM.of(0);

            for (Motor motor : motors) {
                w = w.plus(motor.getEncoderVelocity());
            }

            return w.div(motors.length);
        }

        @Override
        public double getOutputDutyCycle() {
            double output = 0;

            for (Motor motor : motors) {
                output += motor.getOutputDutyCycle();
            }

            return output/motors.length;
        }


    }

    private static class RealMotor implements Motor {
        private SparkMax motor;

        public RealMotor(String name, int canId, boolean inverted) {
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
            runSpeed(MANIPULATOR.ALGAE_IN_SPEED),
            Commands.race(
                Commands.waitUntil(() -> stalled).andThen(() -> expectGamePiece(true)),
                Commands.waitSeconds(MANIPULATOR.ALGAE_GRIP_CHECK_TIME.in(Seconds)).andThen(() -> expectGamePiece(false))
            )
        );
    }

    private Command hold() {
        return this.run(() -> {
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
        });
    }

    public Command runSpeed(double speed) {
        return this.run(() -> {
            if (!ENABLED_SYSTEMS.MANIPULATOR) {
                motors.setDutyCycle(0);
                return;
            }
            
            motors.setDutyCycle(speed);
        });
    }

    public Command intake() {
        return this.runSpeed(MANIPULATOR.ALGAE_IN_SPEED).until(() -> stalled).andThen(() -> expectGamePiece(true)); // do finallydo?
    }

    public Command drop() {
        return this.runSpeed(-MANIPULATOR.ALGAE_OUT_SPEED).until(() -> !stalled).finallyDo(() -> {expectGamePiece(false); stopMotors();});
    }

    public Command action() {
        return Commands.defer(() -> hasGamePiece() ? drop() : intake(), Set.of(this));
    }

    public Command stop() {
        return Commands.run(() -> motors.setDutyCycle(0), this);
    }

    public void stopMotors() {
        motors.setDutyCycle(0);
    }

    @Override
    public void periodic() {
        // stalled = stallDebouncer.calculate(
        //     (leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent())/2.0 > MANIPULATOR.ALGAE_DETECT_CURRENT.in(Amps)
        // );
        stalled = stallDebouncer.calculate(motors.getEncoderVelocity().abs(RPM) < 1.0 && Math.abs(motors.getOutputDutyCycle()) > 0.0);
    }
}
