package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.PIDConstants;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.SwerveConfig.Chassis;
import com.team6962.lib.swerve.SwerveConfig.DriveGains;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Module;
import com.team6962.lib.swerve.SwerveConfig.Motor;
import com.team6962.lib.swerve.SwerveConfig.Wheel;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Preferences;

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
    ChassisType chassisType = getChassisType();

    return new SwerveConfig(
        getChassis(chassisType),
        Gearing.MK4I_L2_PLUS,
        getModules(chassisType),
        new Motor(
            DCMotor.getKrakenX60(1),
            new Slot0Configs()
                .withKP(0.01)
                .withKD(0.01)
                .withKI(0.1)
                .withKV(0.117)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign),
            Amps.of(60)),
        new Motor(
            DCMotor.getKrakenX60(1),
            new Slot0Configs()
                .withKP(20)
                .withKI(1)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign),
            Amps.of(60)),
        Wheel.COLSON,
        new DriveGains(
            new PIDConstants(3.0, 0.0, 0.5), new PIDConstants(1.0, 0.5, 0.01))
          .withFineTranslation(new PIDConstants(3.0, 0.0, 0.5)))
      .withMaxDriveSpeed(MetersPerSecond.of(3.9))
      .withMaxRotationSpeed(RotationsPerSecond.of(3.1));
  }

  private static Chassis getChassis(ChassisType chassisType) {
    return switch (chassisType) {
      case COMPETITION -> new Chassis(
          Inches.of(36), Inches.of(36), Inches.of(24.75), Inches.of(24.75), Pounds.of(135));
      case TEST -> new Chassis(
          Inches.of(28), Inches.of(28), Inches.of(22.75), Inches.of(22.75), Pounds.of(50));
    };
  }

  private static enum ChassisType {
    COMPETITION,
    TEST
  }

  private static ChassisType getChassisType() {
    String idString = Preferences.getString("Chassis", "COMPETITION");

    if (idString.equals("TEST")) {
      System.out.println(
          "=== !!! ### TEST CHASSIS ### !!! ===");
      return ChassisType.TEST;
    } else if (idString.equals("COMPETITION")) {
      System.out.println(
          "=== COMPETITION CHASSIS ===");

      return ChassisType.COMPETITION;
    } else {
      System.out.println("Bad chassis id. Default to");
      System.out.println(
          "=== COMPETITION CHASSIS ===");

      return ChassisType.COMPETITION;
    }
  }

  private static Module[] getModules(ChassisType chassisType) {
    return switch (chassisType) {
      case COMPETITION -> new Module[] {
        Constants.SWERVE.MODULE_CONFIGS[5],
        Constants.SWERVE.MODULE_CONFIGS[4],
        Constants.SWERVE.MODULE_CONFIGS[7],
        Constants.SWERVE.MODULE_CONFIGS[6]
      };
      case TEST -> new Module[] {
        Constants.SWERVE.MODULE_CONFIGS[0],
        Constants.SWERVE.MODULE_CONFIGS[3],
        Constants.SWERVE.MODULE_CONFIGS[1],
        Constants.SWERVE.MODULE_CONFIGS[2]
      };
    };
  }
}
// case 0 -> "Front Left";
// case 1 -> "Front Right";
// case 2 -> "Back Left";
// case 3 -> "Back Right";
