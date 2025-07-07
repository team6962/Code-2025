package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.Supplier;

import com.team6962.lib.telemetry.Logger;
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
         * Constructor for the elevator configuration.
         * @param kP Proportional term of the PID controller
         * @param kI Integral term of the PID controller
         * @param kD Derivative term of the PID controller
         * @param kS Static Feedforward term
         * @param kG Gravity Feedforward term
         * @param kV Velocity Feedforward term
         * @param kA Acceleration Feedforward term
         * @param maxUpwardVelocity Maximum velocity at which elevator can travel upward
         * @param maxUpwardAcceleration Maximum upward acceleration that the elevator can achieve
         * @param maxDownwardVelocity Maximum velocity at which elevator can travel downward
         * @param maxDownwardAcceleration Maximum downward acceleration that the elevator can achieve
         * @param minHeight Minimum (lowest) position of the elevator
         * @param maxHeight Maximum (highest) position of the elevator
         * @param gearReduction Gear reduction from the motor to the sprocket
         * @param sprocketRadius Radius of the sprocket
         * @param motors Array of motors used to drive the elevator
         * @param freeCurrentLimit SPARK MAXs' free current limit
         * @param stallCurrentLimit SPARK MAXs' stall current limit
         * @param tolerance Distance that the elevator must be within to be considered at a target position
         * @param floorLimitSwitch The configuration for the floor limit switch
         * @param ceilingLimitSwitch The configuration for the ceiling limit switch
         */
        public Config(
            double kP,
            double kI,
            double kD,
            double kS,
            double kG,
            double kV,
            double kA,
            LinearVelocity maxUpwardVelocity,
            LinearAcceleration maxUpwardAcceleration,
            LinearVelocity maxDownwardVelocity,
            LinearAcceleration maxDownwardAcceleration,
            Distance minHeight,
            Distance maxHeight,
            double gearReduction,
            Distance sprocketRadius,
            MotorConfig[] motors,
            Current freeCurrentLimit,
            Current stallCurrentLimit,
            Distance tolerance,
            LimitSwitchConfig floorLimitSwitch,
            LimitSwitchConfig ceilingLimitSwitch
        ) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kS = kS;
            this.kG = kG;
            this.kV = kV;
            this.kA = kA;
            this.maxUpwardVelocity = maxUpwardVelocity;
            this.maxUpwardAcceleration = maxUpwardAcceleration;
            this.maxDownwardVelocity = maxDownwardVelocity;
            this.maxDownwardAcceleration = maxDownwardAcceleration;
            this.minHeight = minHeight;
            this.maxHeight = maxHeight;
            this.gearReduction = gearReduction;
            this.sprocketRadius = sprocketRadius;
            this.motors = motors;
            this.freeCurrentLimit = freeCurrentLimit;
            this.stallCurrentLimit = stallCurrentLimit;
            this.tolerance = tolerance;
            this.floorLimitSwitch = floorLimitSwitch;
            this.ceilingLimitSwitch = ceilingLimitSwitch;
        }

        /**
         * Proportional gain of the PID controller.
         */
        public double kP;

        /**
         * Integral gain of the PID controller.
         */
        public double kI;

        /**
         * Derivative gain of the PID controller.
         */
        public double kD;

        /**
         * Static feedforward gain, which is the voltage required for the
         * elevator to overcome static friction.
         */
        public double kS;

        /**
         * Gravity feedforward gain, which is the voltage required to
         * overcome the force of gravity on the elevator.
         */
        public double kG;

        /**
         * Velocity feedforward gain, which is the coefficient multiplied
         * by velocity to find the voltage required to maintain that velocity.
         */
        public double kV;

        /**
         * Acceleration feedforward gain, which is the coefficient
         * multiplied by acceleration to find the voltage required to reach
         * that acceleration.
         */
        public double kA;

        /**
         * Maximum velocity at which elevator can travel updward.
         */
        public LinearVelocity maxUpwardVelocity;

        /**
         * Maximum upward acceleration that the elevator can achieve.
         */
        public LinearAcceleration maxUpwardAcceleration;

        /**
         * Maximum velocity at which elevator can travel downward.
         */
        public LinearVelocity maxDownwardVelocity;

        /**
         * Maximum downward acceleration that the elevator can achieve.
         */
        public LinearAcceleration maxDownwardAcceleration;

        /**
         * Minimum (lowest) position of the elevator.
         */
        public Distance minHeight;

        /**
         * Maximum (highest) position of the elevator.
         */
        public Distance maxHeight;

        /**
         * Gear reduction from the motor to the sprocket.
         */
        public double gearReduction;

        /**
         * Radius of the sprocket attached to the chain that is directly
         * connected to the carriage.
         */
        public Distance sprocketRadius;

        /**
         * Array of motors that are used to drive the elevator.
         */
        public MotorConfig[] motors;

        /**
         * SPARK MAXs' free current limit.
         */
        public Current freeCurrentLimit;

        /**
         * SPARK MAXs' stall current limit.
         */
        public Current stallCurrentLimit;

        /**
         * Distance that the elevator must be within to be considered at
         * a target position.
         */
        public Distance tolerance;

        /**
         * The configuration for the floor limit switch.
         */
        public LimitSwitchConfig floorLimitSwitch;

        /**
         * The configuration for the ceiling limit switch.
         */
        public LimitSwitchConfig ceilingLimitSwitch;

        /**
         * Robot control loop cycle frequency (frequency that periodic is
         * called).
         */
        public Frequency updateFrequency = Hertz.of(50);
    }

    /**
     * Configuration for a motor used in the BasedElevator.
     */
    public static class MotorConfig {
        /**
         * Name of the motor (e.g. left or right), used for logging.
         */
        public String name;

        /**
         * CAN ID of the motor.
         */
        public int canId;

        /**
         * Whether the motor is inverted or not. If true, the motor will
         * run in the opposite direction of the setpoint.
         */
        public boolean inverted;

        /**
         * Constructor for the motor configuration.
         * @param name Name of the motor, used for logging.
         * @param canId CAN ID of the motor.
         * @param inverted Whether the motor is inverted or not.
         */
        public MotorConfig(String name, int canId, boolean inverted) {
            this.name = name;
            this.canId = canId;
            this.inverted = inverted;
        }
    }

    /**
     * Configuration for a limit switch used in the BasedElevator.
     */
    public static class LimitSwitchConfig {
        /**
         * Digital Input channel for the limit switch.
         */
        public int dio;
        
        /**
         * Wiring of the limit switch, whether it is normally open or normally
         * closed.
         */
        public LimitSwitchWiring wiring;

        /**
         * Whether the limit switch should seed the absolute position of the
         * elevator when pressed.
         */
        public boolean shouldSeedPosition;

        /**
         * Constructor for the limit switch configuration.
         * @param dio Digital Input channel for the limit switch.
         * @param wiring Wiring of the limit switch, whether it is normally open or normally closed.
         * @param shouldSeedPosition Whether the limit switch should seed the absolute position of the elevator when pressed.
         */
        public LimitSwitchConfig(int dio, LimitSwitchWiring wiring, boolean shouldSeedPosition) {
            this.dio = dio;
            this.wiring = wiring;
            this.shouldSeedPosition = shouldSeedPosition;
        }
    }

    /**
     * Enumeration for the possible wiring configurations of a limit switch.
     */
    public static enum LimitSwitchWiring {
        /**
         * The limit switch is normally open, meaning that the circuit is open
         * when the switch is not pressed,
         */
        NormallyOpen,

        /**
         * The limit switch is normally closed, meaning that the circuit is
         * closed when the switch is not pressed.
         */
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

    /**
     * Create a BasedElevator with the given configuration.
     * @param config The configuration for the elevator, which includes PID and
     * feedforward gains, motor configurations, limit switch configurations, and
     * elevator dimensions.
     */
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

        Logger.logMeasure(getName() + "/position", this::getPosition);
        Logger.logMeasure(getName() + "/velocity", this::getVelocity);
        Logger.logBoolean(getName() + "/seeded", () -> absolutePositionSeeded);

        floor.logUnder(getName() + "/limits/floor");
        ceiling.logUnder(getName() + "/limits/ceiling");
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

    public Command stop() {
        return run(this::stopMotors);
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
            motor.run(feedbackTarget, feedforwardVoltage);
        }
    }

    public Command applyVoltage(Supplier<Voltage> voltageSupplier) {
        return run(() -> {
            Voltage voltage = voltageSupplier.get();

            if (safeToMoveInDirection(voltage.in(Volts))) {
                for (BasedMotor motor : motors) {
                    motor.setVoltage(voltage);
                }
            } else {
                stopMotors();
            }
        });
    }

    public Command applyVoltage(Voltage voltage) {
        return applyVoltage(() -> voltage);
    }

    public Command applyDutyCycle(double dutyCycle) {
        return applyVoltage(() -> Volts.of(dutyCycle * 12.0));
    }

    public Command moveSpeed(LinearVelocity targetVelocity) {
        return applyVoltage(() -> Volts.of(feedforward.calculate(
            targetVelocity.in(MetersPerSecond)
        )));
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

    public boolean atPosition(Distance position) {
        return getPosition().minus(position).abs(Meters) < config.tolerance.in(Meters);
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
