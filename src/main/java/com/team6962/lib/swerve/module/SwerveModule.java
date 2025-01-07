package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SwerveModule extends SubsystemBase {
    /**
     * The drive motor for this module.
     */
    private TalonFX driveMotor;

    /**
     * The steer motor for this module.
     */
    private TalonFX steerMotor;

    /**
     * The steer encoder for this module, whose position is directly lined up
     * with the wheel. This means that the encoder outputs values in mechanism
     * rotations.
     */
    private CANcoder steerEncoder;

    private SwerveConfig constants;
    private SwerveModule.Corner corner;

    protected boolean isCalibrating = false;

    public void configureModule(SwerveConfig config, SwerveModule.Corner corner) {
        this.constants = config;
        this.corner = corner;

        // Get this swerve module's configuration
        SwerveConfig.Module moduleConstants = getModuleConstants();

        // Connect to the module's drive motor
        driveMotor = new TalonFX(moduleConstants.driveMotorId());

        // Get the 'configurator' for the drive motor, which allows us to
        // configure the motor's settings
        TalonFXConfigurator driveConfig = driveMotor.getConfigurator();

        // Apply the PID/feedforward/Motion Magic configuration given in the
        // swerve drive configuration to the drive motor
        CTREUtils.check(driveConfig.apply(config.driveMotor().gains()));

        // Configure the drive motor to brake automatically when not driven
        CTREUtils.check(driveConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)));

        // Set the drive motor gear ratio to the value given in the swerve drive
        // configuration
        CTREUtils.check(driveConfig.apply(new FeedbackConfigs().withSensorToMechanismRatio(config.gearing().drive())));

        // Connect to the module's steer encoder
        steerEncoder = new CANcoder(moduleConstants.steerEncoderId());

        // Get the 'configurator' for the steer encoder, which allows us to
        // configure the encoder's settings
        CANcoderConfigurator steerEncoderConfig = steerEncoder.getConfigurator();

        // Set the absolute steer encoder offset to the value given in the
        // swerve module's configuration
        CTREUtils.check(steerEncoderConfig.apply(new MagnetSensorConfigs()
            .withMagnetOffset(moduleConstants.steerEncoderOffset().in(Rotations))));

        // Connect to the module's steer motor
        steerMotor = new TalonFX(moduleConstants.steerMotorId());

        // Get the 'configurator' for the steer motor, which allows us to
        // configure the motor's settings
        TalonFXConfigurator steerConfig = steerMotor.getConfigurator();

        // Apply the PID/feedforward/Motion Magic configuration given in the
        // swerve drive configuration to the steer motor
        CTREUtils.check(steerConfig.apply(config.steerMotor().gains()));

        // Configure the steer motor to be inverted, and brake automatically
        // when not driven
        CTREUtils.check(steerConfig.apply(new MotorOutputConfigs()
            //.withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)));
        
        // Configure the fusing of the absolute steer encoder's reported position
        // with the motor's internal relative encoder, and set the steer motor
        // gear ratio to the value given in the swerve drive configuration
        CTREUtils.check(steerConfig.apply(new FeedbackConfigs()
            .withFusedCANcoder(steerEncoder)
            .withRotorToSensorRatio(config.gearing().steer())
            .withSensorToMechanismRatio(1)));
    }

    /**
     * Gets the drive motor for this module.
     * @return The TalonFX drive motor controller
     */
    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    /**
     * Gets the steer motor for this module.
     * @return The TalonFX steer motor controller
     */
    public TalonFX getSteerMotor() {
        return steerMotor;
    }

    /**
     * Gets the steer encoder for this module.
     * @return The steer CANcoder
     */
    public CANcoder getSteerEncoder() {
        return steerEncoder;
    }

    public SwerveConfig getDrivetrainConstants() {
        return constants;
    }

    public SwerveConfig.Module getModuleConstants() {
        return constants.module(corner);
    }

    public Corner getModuleCorner() {
        return corner;
    }

    public void driveState(SwerveModuleState targetState) {
        if (isCalibrating) return;

        targetState.optimize(getState().angle);

        CTREUtils.check(driveMotor.setControl(new VelocityTorqueCurrentFOC(targetState.speedMetersPerSecond)));
        CTREUtils.check(steerMotor.setControl(new PositionVoltage(targetState.angle.getRotations())));
    }

    public Distance getDrivePosition() {
        return constants.wheel().diameter().times(CTREUtils.unwrap(driveMotor.getPosition()).in(Radians));
    }

    public LinearVelocity getDriveSpeed() {
        return MetersPerSecond.of(constants.wheel().radius().in(Meters) * CTREUtils.unwrap(driveMotor.getVelocity()).in(RadiansPerSecond));
    }

    public Angle getSteerAngle() {
        return CTREUtils.unwrap(steerEncoder.getPosition());
    }

    public AngularVelocity getSteerVelocity() {
        return CTREUtils.unwrap(steerEncoder.getVelocity());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getSteerAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerAngle()));
    }

    public Current getConsumedCurrent() {
        return CTREUtils.unwrap(driveMotor.getSupplyCurrent()).plus(CTREUtils.unwrap(steerMotor.getSupplyCurrent()));
    }

    public Pose2d getRelativePose() {
        return new Pose2d(
            calculateRelativeTranslation(corner.index, constants.chassis()),
            new Rotation2d(getSteerAngle())
        );
    }

    public Command calibrateSteerMotor(Voltage averageBusVoltage, Current maxCurrent) {
        return calibrateMotor("steer", getSteerMotor(), averageBusVoltage, maxCurrent, log ->
            log.angularPosition(getSteerAngle()).angularVelocity(getSteerVelocity()));
    }

    public Command calibrateDriveMotor(Voltage averageBusVoltage, Current maxCurrent) {
        return calibrateMotor("drive", getDriveMotor(), averageBusVoltage, maxCurrent, log ->
            log.linearPosition(getDrivePosition()).linearVelocity(getDriveSpeed()));
    }

    private Command calibrateMotor(String motorName, TalonFX motor, Voltage averageBusVoltage, Current maxCurrent, Consumer<MotorLog> logEncoder) {
        SysIdRoutine calibrationRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> motor.setControl(new TorqueCurrentFOC(voltage.in(Volts))),
                log -> logEncoder.accept(
                    log.motor("swerve-module-" + SwerveModule.getModuleSysIdName(getModuleCorner().index) + "-" + motorName)
                        .voltage(CTREUtils.unwrap(motor.getMotorVoltage()))
                ),
                this
            )
        );

        return Commands.sequence(
            Commands.runOnce(() -> isCalibrating = true),
            Commands.waitSeconds(1.0),
            calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.runOnce(() -> motor.stopMotor()),
            Commands.waitSeconds(1.0),
            calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(() -> motor.stopMotor()),
            Commands.waitSeconds(1.0),
            calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.runOnce(() -> motor.stopMotor()),
            Commands.waitSeconds(1.0),
            calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(() -> motor.stopMotor()),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> isCalibrating = false)
        );
    }

    public static String getModuleName(int moduleIndex) {
        return switch (moduleIndex) {
            case 0 -> "Front Left";
            case 1 -> "Front Right";
            case 2 -> "Back Left";
            case 3 -> "Back Right";
            default -> throw new IllegalArgumentException("Invalid module index");
        };
    }

    public static String getModuleSysIdName(int moduleIndex) {
        return switch (moduleIndex) {
            case 0 -> "front-left";
            case 1 -> "front-right";
            case 2 -> "back-left";
            case 3 -> "back-right";
            default -> throw new IllegalArgumentException("Invalid module index");
        };
    }
    
    public static enum Corner {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        BACK_LEFT(2),
        BACK_RIGHT(3);

        public final int index;

        private Corner(int index) {
            this.index = index;
        }

        public static Corner fromIndex(int index) {
            return switch (index) {
                case 0 -> FRONT_LEFT;
                case 1 -> FRONT_RIGHT;
                case 2 -> BACK_LEFT;
                case 3 -> BACK_RIGHT;
                default -> throw new IllegalArgumentException("Invalid module index");
            };
        }
    }

    public static Translation2d calculateRelativeTranslation(int cornerIndex, SwerveConfig.Chassis chassis) {
        return new Translation2d(
            (cornerIndex >= 2 ? -1 : 1) * chassis.wheelBase().in(Meters) / 2,
            (cornerIndex % 2 == 1 ? -1 : 1) * chassis.trackWidth().in(Meters) / 2
        );
    }
}
