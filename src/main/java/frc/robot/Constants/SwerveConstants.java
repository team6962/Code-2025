package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

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
import com.team6962.lib.telemetry.Logger;

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
        new DriveGains( // TODO: Tune drive gains
            new PIDConstants(5.0, 1.0, 0), new PIDConstants(5.0, 1.0, 0)));
  }

  private static Chassis getChassis(ChassisType chassisType) {
    return switch (chassisType) {
      case COMPETITION -> new Chassis(
          Inches.of(30), Inches.of(30), Inches.of(24.75), Inches.of(24.75), Pounds.of(135));
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
          "████████╗███████╗░██████╗████████╗  ░█████╗░██╗░░██╗░█████╗░░██████╗░██████╗██╗░██████╗██╗\n"
              + "╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝  ██╔══██╗██║░░██║██╔══██╗██╔════╝██╔════╝██║██╔════╝██║\n"
              + "░░░██║░░░█████╗░░╚█████╗░░░░██║░░░  ██║░░╚═╝███████║███████║╚█████╗░╚█████╗░██║╚█████╗░██║\n"
              + "░░░██║░░░██╔══╝░░░╚═══██╗░░░██║░░░  ██║░░██╗██╔══██║██╔══██║░╚═══██╗░╚═══██╗██║░╚═══██╗╚═╝\n"
              + "░░░██║░░░███████╗██████╔╝░░░██║░░░  ╚█████╔╝██║░░██║██║░░██║██████╔╝██████╔╝██║██████╔╝██╗\n"
              + "░░░╚═╝░░░╚══════╝╚═════╝░░░░╚═╝░░░  ░╚════╝░╚═╝░░╚═╝╚═╝░░╚═╝╚═════╝░╚═════╝░╚═╝╚═════╝░╚═╝");
      return ChassisType.TEST;
    } else if (idString.equals("COMPETITION")) {
      System.out.println(
          "█▀▀ █▀█ █▀▄▀█ █▀█ █▀▀ ▀█▀ █ ▀█▀ █ █▀█ █▄░█ █\n"
              + "█▄▄ █▄█ █░▀░█ █▀▀ ██▄ ░█░ █ ░█░ █ █▄█ █░▀█ ▄");

      return ChassisType.COMPETITION;
    } else {
      System.out.println("Bad chassis id. Default to");
      System.out.println(
          "█▀▀ █▀█ █▀▄▀█ █▀█ █▀▀ ▀█▀ █ ▀█▀ █ █▀█ █▄░█ █\n"
              + "█▄▄ █▄█ █░▀░█ █▀▀ ██▄ ░█░ █ ░█░ █ █▄█ █░▀█ ▄");

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
        Constants.SWERVE.MODULE_CONFIGS[1],
        Constants.SWERVE.MODULE_CONFIGS[2],
        Constants.SWERVE.MODULE_CONFIGS[3]
      };
    };
  }
}
