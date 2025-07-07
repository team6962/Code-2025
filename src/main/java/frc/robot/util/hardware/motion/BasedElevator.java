package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;

import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasedElevator extends SubsystemBase {
    /**
     * Configuration for a BasedElevator.
     */
    public static class Config {
        /**
         * The Proportional term of the PID controller.
         */
        public double kP;

        /**
         * The Integral term of the PID controller.
         */
        public double kI;

        /**
         * The Derivative term of the PID controller.
         */
        public double kD;

        /**
         * The Static Feedforward term, which is the voltage required for the
         * elevator to overcome static friction.
         */
        public double kS;

        /**
         * The Gravity Feedforward term, which is the voltage required to
         * overcome the force of gravity on the elevator.
         */
        public double kG;

        /**
         * The Velocity Feedforward term, which is the coefficient multiplied
         * by velocity to find the voltage required to maintain that velocity.
         */
        public double kV;

        /**
         * The Acceleration Feedforward term, which is the coefficient
         * multiplied by acceleration to find the voltage required to reach
         * that acceleration.
         */
        public double kA;

        /**
         * The maximum velocity that the elevator can travel updward.
         */
        public LinearVelocity maxUpwardVelocity;

        /**
         * The maximum upward acceleration that the elevator can achieve.
         */
        public LinearAcceleration maxUpwardAcceleration;

        /**
         * The maximum velocity that the elevator can travel downward.
         */
        public LinearVelocity maxDownwardVelocity;

        /**
         * The maximum downward acceleration that the elevator can achieve.
         */
        public LinearAcceleration maxDownwardAcceleration;

        public Distance minHeight;
        public Distance maxHeight;

        public double gearReduction;
        public Distance sprocketRadius;

        public MotorConfig[] motors;

        public Current freeCurrentLimit;
        public Current stallCurrentLimit;

        public Distance tolerance;

        public LimitSwitchConfig floorLimitSwitch;
        public LimitSwitchConfig ceilingLimitSwitch;

        public Frequency updateFrequency = Hertz.of(50);
    }

    public static class MotorConfig {
        public String name;
        public int canId;
        public boolean inverted;
    }

    public static class LimitSwitchConfig {
        public int dio;
        public LimitSwitchWiring wiring;
        public boolean shouldSeedPosition;
    }

    public static enum LimitSwitchWiring {
        NormallyOpen,
        NormallyClosed
    }

    private Config config;

    private BasedMotor[] motors;
    private BasedLimitSwitch floor;
    private BasedLimitSwitch ceiling;

    private ElevatorFeedforward feedforward;
    private TrapezoidProfile downProfile;
    private TrapezoidProfile upProfile;

    private boolean absolutePositionSeeded = false;

    public BasedElevator(Config config) {
        this.config = config;

        motors = new BasedMotor[config.motors.length];
        for (int i = 0; i < config.motors.length; i++) {
            motors[i] = new BasedMotor(config, config.motors[i]);
        }

        floor = new BasedLimitSwitch(config.floorLimitSwitch);
        ceiling = new BasedLimitSwitch(config.ceilingLimitSwitch);

        feedforward = new ElevatorFeedforward(config.kS, config.kG, config.kV, config.kA);

        upProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                config.maxUpwardVelocity.in(MetersPerSecond),
                config.maxUpwardAcceleration.in(MetersPerSecondPerSecond)
            )
        );

        downProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                config.maxDownwardVelocity.in(MetersPerSecond),
                config.maxDownwardAcceleration.in(MetersPerSecondPerSecond)
            )
        );
    }

    public Distance getPosition() {
        double positionSum = 0;

        for (BasedMotor motor : motors) {
            positionSum += motor.getPosition().in(Meters);
        }

        return Meters.of(positionSum / motors.length);
    }

    public LinearVelocity getVelocity() {
        double velocitySum = 0;

        for (BasedMotor motor : motors) {
            velocitySum += motor.getVelocity().in(MetersPerSecond);
        }

        return MetersPerSecond.of(velocitySum / motors.length);
    }

    private void stopMotors() {
        for (BasedMotor motor : motors) {
            motor.stop();
        }
    }

    private void runMotors(Distance feedbackTarget, Voltage feedforwardVoltage) {
        if (
            !safeToMoveInDirection(feedbackTarget.minus(getPosition()).in(Meters)) ||
            !safeToMoveInDirection(feedforwardVoltage.in(Volts))
        ) {
            stopMotors();
        }

        for (BasedMotor motor : motors) {
            motor.run(feedbackTarget.in(Meters), feedforwardVoltage);
        }
    }

    private Command moveTo(Distance targetPosition, LinearVelocity targetVelocity, boolean endWithinTolerance) {
        final Distance clampedTarget = MeasureMath.clamp(targetPosition, config.minHeight, config.maxHeight);

        return Commands.defer(() -> new Command() {
            private TrapezoidProfile.State initialState;

            private TrapezoidProfile.State goalState = new TrapezoidProfile.State(
                clampedTarget.in(Meters), targetVelocity.in(MetersPerSecond)
            );

            private double startTime;

            @Override
            public void initialize() {
                initialState = new TrapezoidProfile.State(
                    getPosition().in(Meters), getVelocity().in(MetersPerSecond)
                );

                startTime = Timer.getFPGATimestamp();
            }

            @Override
            public void execute() {
                double currentTime = Timer.getFPGATimestamp() - startTime;

                TrapezoidProfile profile = goalState.position < initialState.position ? downProfile : upProfile;

                TrapezoidProfile.State currentState = profile.calculate(
                    currentTime,
                    initialState,
                    goalState
                );

                TrapezoidProfile.State nextState = profile.calculate(
                    currentTime + config.updateFrequency.asPeriod().in(Seconds),
                    initialState,
                    goalState
                );

                Voltage feedforwardVoltage = Volts.of(feedforward.calculateWithVelocities(currentState.velocity, nextState.velocity));

                runMotors(Meters.of(currentState.position), feedforwardVoltage);
            }

            @Override
            public boolean isFinished() {
                return endWithinTolerance && getPosition().minus(clampedTarget).abs(Meters) < config.tolerance.in(Meters);
            }
        }, Set.of(this));
    }

    public Command moveTo(Distance targetPosition, LinearVelocity targetVelocity) {
        return moveTo(targetPosition, targetVelocity, true);
    }

    public Command moveTo(Distance targetPosition) {
        return moveTo(targetPosition, MetersPerSecond.of(0), true);
    }

    public Command holdAt(Distance targetPosition) {
        return moveTo(targetPosition, MetersPerSecond.of(0), false);
    }

    public Command hold() {
        return Commands.defer(() -> holdAt(getPosition()), Set.of(this));
    }

    private void setPosition(Distance position) {
        for (BasedMotor motor : motors) {
            motor.setPosition(position);
        }
    }

    private boolean safeToMoveInDirection(double direction) {
        if (direction > 0 && (ceiling.isPressed() || !absolutePositionSeeded)) {
            return false;
        } else if (direction < 0 && floor.isPressed()) {
            return false;
        }

        return true;
    }

    @Override
    public void periodic() {
        Distance absolutePosition = null;

        if (config.floorLimitSwitch.shouldSeedPosition && floor.isPressed()) {
            absolutePosition = config.minHeight;
        } else if (config.ceilingLimitSwitch.shouldSeedPosition && ceiling.isPressed()) {
            absolutePosition = config.maxHeight;
        }

        if (absolutePosition != null) {
            absolutePositionSeeded = true;

            setPosition(absolutePosition);
        }
    }
}
