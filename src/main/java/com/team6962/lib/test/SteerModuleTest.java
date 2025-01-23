package com.team6962.lib.test;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SteerModuleTest extends SubsystemBase {
    private static final double rotorToSensorReduction = SwerveConfig.Gearing.MK4I_L2_PLUS.steer();
    private static final SwerveConfig.Wheel wheel = SwerveConfig.Wheel.COLSON;
    
    private TalonFX steerMotor;
    private CANcoder encoder;

    private FlywheelSim steerMotorSim;

    private Angle wheelAngle = Rotations.of(0);

    public SteerModuleTest() {
        encoder = new CANcoder(11);

        CANcoderConfigurator encoderConfig = encoder.getConfigurator();

        CTREUtils.check(encoderConfig.apply(new MagnetSensorConfigs()
            .withMagnetOffset(0.25)));

        steerMotor = new TalonFX(10);

        TalonFXConfigurator steerConfig = steerMotor.getConfigurator();

        CTREUtils.check(steerConfig.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)));

        CTREUtils.check(steerConfig.apply(new FeedbackConfigs()
            .withFusedCANcoder(encoder)
            .withRotorToSensorRatio(rotorToSensorReduction)));
        
        CTREUtils.check(steerConfig.apply(new Slot0Configs()
            .withKP(-1000.0)));
        
        steerMotorSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                wheel.steerMomentOfInertia().in(KilogramSquareMeters),
                rotorToSensorReduction
            ),
            DCMotor.getKrakenX60Foc(1)
        );

        // CTREUtils.check(steerConfig.apply(config.steerMotor().gains()));

        // // Configure the steer motor to be inverted, and brake automatically
        // // when not driven
        // CTREUtils.check(steerConfig.apply(new MotorOutputConfigs()
        //     //.withInverted(InvertedValue.Clockwise_Positive)
        //     .withNeutralMode(NeutralModeValue.Brake)));
        
        // // Configure the fusing of the absolute steer encoder's reported position
        // // with the motor's internal relative encoder, and set the steer motor
        // // gear ratio to the value given in the swerve drive configuration
        // CTREUtils.check(steerConfig.apply(new FeedbackConfigs()
        //     .withFusedCANcoder(steerEncoder)
        //     .withRotorToSensorRatio(config.gearing().steer())
        //     .withSensorToMechanismRatio(1)));
    }

    @Override
    public void periodic() {
        Time delta = Seconds.of(0.02);

        TalonFXSimState steerSimState = steerMotor.getSimState();

        steerMotorSim.setInputVoltage(steerSimState.getMotorVoltage());
        steerMotorSim.update(delta.in(Seconds));

        wheelAngle = wheelAngle.plus(steerMotorSim.getAngularVelocity().times(delta));

        CTREUtils.check(steerSimState.setRawRotorPosition(wheelAngle.times(rotorToSensorReduction)));

        CANcoderSimState encoderSimState = encoder.getSimState();

        CTREUtils.check(encoderSimState.setRawPosition(wheelAngle));

        Rotation2d position = new Rotation2d(CTREUtils.unwrap(steerMotor.getPosition()));

        Logger.getField().getObject("Swerve Module Test").setPose(new Pose2d(5, 5, position));

        steerMotor.setControl(new PositionTorqueCurrentFOC(Rotations.of(0.5)));
    }
}
