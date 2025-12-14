package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.PIDConstants;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.SwerveConfig.Chassis;
import com.team6962.lib.swerve.SwerveConfig.DriveGains;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Gyroscope;
import com.team6962.lib.swerve.SwerveConfig.Module;
import com.team6962.lib.swerve.SwerveConfig.Motor;
import com.team6962.lib.swerve.SwerveConfig.Wheel;

import edu.wpi.first.math.system.plant.DCMotor;

public final class SwerveConstants {
  private SwerveConstants() {}

  private static SwerveConfig config;

  public static SwerveConfig get() {
    if (config == null) {
      config = generateConfiguration();
    }

    return config;
  }

  private static SwerveConfig generateConfiguration() {
    return new SwerveConfig(
            new Chassis(Inches.of(28), Inches.of(28), Inches.of(22.75), Inches.of(22.75), Pounds.of(115)),
            Gearing.MK4I_L2,
            new Module[] {
              new Module(0, 2, 2, Radians.of(0.05)), // -0.05 rads
              new Module(3, 1, 0, Radians.of(0.049 + Math.PI)), // -0.049 rads
              new Module(5, 7, 1, Radians.of(2.4)), // -2.4 rads
              new Module(4, 6, 3, Radians.of(0.322 + Math.PI)) // -0.322 radsaqwaaqqaq
            },
            new Motor(
                DCMotor.getKrakenX60(1),
                new Slot0Configs()
                    .withKP(0.1)
                    .withKI(0.01)
                    .withKD(0.01)
                    .withKV(0.118)
                    .withKA(0.003933)
                    .withKS(0.17)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign),
                Amps.of(250)),
            new Motor(
                DCMotor.getKrakenX60(1),
                new Slot0Configs()
                    .withKS(0.282)
                    .withKV(2.6)
                    .withKA(0.03)
                    .withKP(18.592)
                    .withKI(0.0)
                    .withKD(0.972)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign),
                Amps.of(60)),
            Wheel.BILLET.withDiameter(Inches.of(3.9053)),
            new DriveGains(new PIDConstants(0.5, 0.0, 0.1), new PIDConstants(0.25, 0.0, 0.05))
                .withFineTranslation(new PIDConstants(3.5, 0.0, 1.5))
                .withFineRotation(new PIDConstants(3.5, 0.0, 1.5)),
            new Gyroscope(0).withMountPose(new MountPoseConfigs().withMountPoseYaw(Degrees.of(90)))
          )
            .withMaxDriveSpeed(MetersPerSecond.of(4.474))
            .withMaxLinearAcceleration(MetersPerSecondPerSecond.of(6.579))
            .withMaxRotationSpeed(RotationsPerSecond.of(1.62))
            .withMaxAngularAcceleration(RotationsPerSecondPerSecond.of(1.28))
            .withCANBus("drivetrain");
  }
}
