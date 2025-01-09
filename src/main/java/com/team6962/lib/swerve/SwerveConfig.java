package com.team6962.lib.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.NewtonMeter;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Ohms;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.team6962.lib.swerve.module.SwerveModule.Corner;
import com.team6962.lib.utils.KinematicsUtils;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Represents the configuration of a swerve drive
 * <p>
 * Note: Feedforward constant equations come from the "FRC Drivetrain Characterization"
 * paper: https://www.chiefdelphi.com/t/paper-frc-drivetrain-characterization/160915
 */
public class SwerveConfig {
    private final Chassis chassis;
    private final Gearing gearing;
    private final Module[] modules;
    private final Motor driveMotor;
    private final Motor steerMotor;
    private final Wheel wheel;
    private final DriveGains driveGains;

    public SwerveConfig(Chassis chassis, Gearing gearing, Module[] modules, Motor driveMotor, Motor steerMotor, Wheel wheel, DriveGains driveGains) {
        this.chassis = chassis;
        this.gearing = gearing;
        this.modules = modules;
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.wheel = wheel;
        this.driveGains = driveGains;

        driveMotor.gains.kV = 1.0 / maxDriveSpeed().in(MetersPerSecond);
        steerMotor.gains.kV = 1.0 / maxSteerSpeed().in(RadiansPerSecond);
    }

    public Chassis chassis() {
        return chassis;
    }

    public Gearing gearing() {
        return gearing;
    }

    public Module[] modules() {
        return modules;
    }

    public Motor driveMotor() {
        return driveMotor;
    }

    public Motor steerMotor() {
        return steerMotor;
    }

    public Wheel wheel() {
        return wheel;
    }

    public DriveGains driveGains() {
        return driveGains;
    }

    public static record Chassis(Distance outerWidth, Distance outerLength, Distance wheelBase, Distance trackWidth, Mass mass) {
        public Translation2d outerDimensions() {
            return new Translation2d(
                outerWidth.in(Meters),
                outerLength.in(Meters)
            );
        }

        public Translation2d drivetrainDimensions() {
            return new Translation2d(
                wheelBase.in(Meters),
                trackWidth.in(Meters)
            );
        }

        public Distance driveRadius() {
            return Meters.of(Math.hypot(wheelBase.in(Meters), trackWidth.in(Meters)) / 2);
        }
    }

    public static enum Gearing {
        MK4I_L2(6.75, 150.0 / 7.0),
        MK4I_L2_PLUS(5.92, 150.0 / 7.0);

        public final double drive;
        public final double steer;

        private Gearing(double driveRatio, double steerRatio) {
            this.drive = driveRatio;
            this.steer = steerRatio;
        }

        public double drive() {
            return drive;
        }

        public double steer() {
            return steer;
        }
    }

    public static record Module(int driveMotorId, int steerMotorId, int steerEncoderId, Angle steerEncoderOffset) {
    }

    public Module module(Corner corner) {
        return modules[corner.index];
    }

    public static record Motor(DCMotor stats, Slot0Configs gains, Current maxCurrent) {
        public Voltage nominalVoltage() {
            return Volts.of(stats.nominalVoltageVolts);
        }

        public Torque stallTorque() {
            return NewtonMeter.of(stats.stallTorqueNewtonMeters);
        }

        public Current stallCurrent() {
            return Amps.of(stats.stallCurrentAmps);
        }

        public Current freeCurrent() {
            return Amps.of(stats.freeCurrentAmps);
        }

        public AngularVelocity freeSpeedRotor() {
            return RadiansPerSecond.of(stats.freeSpeedRadPerSec);
        }

        public Per<AngularVelocityUnit, VoltageUnit> kVRotor() {
            return RadiansPerSecond.per(Volts).ofNative(stats.KvRadPerSecPerVolt);
        }

        public Per<TorqueUnit, CurrentUnit> kTRotor() {
            return NewtonMeters.per(Amps).ofNative(stats.KtNMPerAmp);
        }

        public Resistance internalResistance() {
            return Ohms.of(stats.rOhms);
        }
    }

    public static enum Wheel {
        COLSON(Inches.of(4), 1.0),
        BILLET_NITRILE(Inches.of(4), 1.0),
        BILLET_PRINTED(Inches.of(4), 1.5);

        public final Distance diameter;
        public final double staticFriction;

        private Wheel(Distance diameter, double staticFriction) {
            this.diameter = diameter;
            this.staticFriction = staticFriction;
        }

        public Distance diameter() {
            return diameter;
        }

        public double staticFriction() {
            return staticFriction;
        }

        public Distance radius() {
            return diameter.divide(2);
        }
    }

    public static record DriveGains(PIDConstants translation, PIDConstants rotation) {
        public PIDController createTranslationController() {
            return new PIDController(translation.kP, translation.kI, translation.kD);
        }

        public PIDController createRotationController() {
            return new PIDController(rotation.kP, rotation.kI, rotation.kD);
        }

        public PPHolonomicDriveController pathController() {
            return new PPHolonomicDriveController(
                translation,
                rotation
            );
        }
    }

    // TODO: [EASY] Consider using conversion functions instead of manual conversion
    public LinearVelocity maxDriveSpeed() {
        return driveMotorRotorToMechanism(driveMotor.freeSpeedRotor());
    }

    public AngularVelocity maxSteerSpeed() {
        return steerMotorRotorToMechanism(steerMotor.freeSpeedRotor());
    }

    /**
     * Gets the path constraints for the robot
     * @return The path constraints for the robot
     */
    public PathConstraints pathConstraints() {
        return new PathConstraints(
            maxDriveSpeed(),
            maxLinearAcceleration(Amps.of(40)),
            maxRotationSpeed(),
            maxAngularAcceleration(Amps.of(40))
        );
    }

    public MomentOfInertia momentOfInertia() {
        Translation2d outer = chassis.outerDimensions();

        return KilogramSquareMeters.of(
            (1 / 12) * chassis.mass.in(Kilograms) * (outer.getX() * outer.getX() + outer.getY() * outer.getY())
        );
    }

    public Force staticFriction() {
        return Newtons.of(wheel.staticFriction * 9.81 * chassis.mass.in(Kilograms));
    }

    public Torque driveTorque(Current drivenCurrent) {
        return NewtonMeter.of(driveMotor.stats.getTorque(drivenCurrent.in(Amps))).times(gearing.drive);
    }

    public LinearAcceleration maxLinearAcceleration(Current drivenCurrent) {
        return MeasureMath.min(driveTorque(drivenCurrent).div(wheel.radius()), staticFriction())
                .div(chassis.mass);
    }

    public AngularVelocity maxRotationSpeed() {
        return RadiansPerSecond.of(maxDriveSpeed().in(MetersPerSecond) / chassis.driveRadius().in(Meters));
    }

    public AngularAcceleration maxAngularAcceleration(Current driveCurrent) {
        double friction = staticFriction().in(Newtons) / chassis.driveRadius().in(Meters) / chassis.mass.in(Kilograms);
        double torque = driveTorque(driveCurrent).in(NewtonMeters) / chassis.driveRadius().in(Meters) / momentOfInertia().in(KilogramSquareMeters);

        return RadiansPerSecond.per(Second).of(Math.min(friction, torque));
    }

    public FlywheelSim createDriveMotorSimulation() {
        return new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                driveMotor.gains.kV * wheel.radius().in(Meters),
                driveMotor.gains.kA * wheel.radius().in(Meters)
            ),
            driveMotor.stats,
            gearing.drive
        );
    }

    public FlywheelSim createSteerMotorSimulation() {
        return new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                steerMotor.gains.kV,
                steerMotor.gains.kA
            ),
            steerMotor.stats,
            gearing.steer
        );
    }

    public Distance driveMotorRotorToMechanism(Angle movement) {
        return Meters.of(movement.in(Radians) * wheel.radius().in(Meters) / gearing.drive);
    }

    public LinearVelocity driveMotorRotorToMechanism(AngularVelocity movement) {
        return MetersPerSecond.of(movement.in(RadiansPerSecond) * wheel.radius().in(Meters) / gearing.drive);
    }

    public Angle driveMotorMechanismToRotor(Distance movement) {
        return Radians.of(movement.in(Meters) * gearing.drive / wheel.radius().in(Meters));
    }

    public AngularVelocity driveMotorMechanismToRotor(LinearVelocity movement) {
        return RadiansPerSecond.of(movement.in(MetersPerSecond) * gearing.drive / wheel.radius().in(Meters));
    }
    
    public Angle steerMotorRotorToMechanism(Angle movement) {
        return movement.divide(gearing.steer);
    }

    public AngularVelocity steerMotorRotorToMechanism(AngularVelocity movement) {
        return movement.divide(gearing.steer);
    }

    public Angle steerMotorMechanismToRotor(Angle movement) {
        return movement.times(gearing.steer);
    }

    public AngularVelocity steerMotorMechanismToRotor(AngularVelocity movement) {
        return movement.times(gearing.steer);
    }

    public RobotConfig pathRobotConfig() {
        return new RobotConfig(
            chassis().mass(),
            momentOfInertia(),
            new ModuleConfig(
                wheel().radius(),
                maxDriveSpeed(),
                wheel().staticFriction(),
                driveMotor().stats(),
                driveMotor().maxCurrent(),
                1
            ),
            KinematicsUtils.modulePositionsFromChassis(chassis())
        );
    }
}
