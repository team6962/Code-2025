// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.SwerveConfig.Chassis;
import com.team6962.lib.swerve.SwerveConfig.DriveGains;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Motor;
import com.team6962.lib.swerve.SwerveConfig.Wheel;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final Supplier<Boolean> IS_BLUE_TEAM = () -> !(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

  // ENABLED SYSTEMS
  public static final class ENABLED_SYSTEMS {
    public static final boolean ENABLE_DRIVE     = true;
    public static final boolean ENABLE_DASHBOARD = true;
    public static final boolean ENABLE_SHOOTER   = true;
    public static final boolean ENABLE_INTAKE   = true;
    public static final boolean ENABLE_TRANSFER = true;
    public static final boolean ENABLE_HANG = false;
    public static final boolean ENABLE_MANIPULATOR = false;
    public static final boolean ENABLE_AMP = true;
  }

  public static final class LOGGING {
    public static final int LOGGING_PERIOD_MS = 20;
  }

  // DEVICES
  public static final class DEVICES {
    public static final int DRIVE_XBOX_CONTROLLER = 0;
    public static final int OPERATOR_XBOX_CONTROLLER = 1;
  }

  // DASHBOARD (ShuffleBoard)
  public static final class DASHBOARD {
    public static final String TAB_NAME = "SwerveDrive";
  }

  public static final class ALGAE {
    public static final double ALGAE_DIAMETER = 16.25; //inches
  }

  // LIMELIGHT
  // Exposure: 750
  // Sensor Gain: 10
  public static final class LIMELIGHT {
    // x is front-to-back
    // y is left-to-right
    // z it top-to-bottom
    public static final Map<String, Pose3d> APRILTAG_CAMERA_POSES = Map.of(
      "limelight-ftag", new Pose3d(Units.inchesToMeters(7.442142), Units.inchesToMeters(1.0), Units.inchesToMeters(25.283), new Rotation3d(0.0, Units.degreesToRadians(24.0), 0.0)),
      "limelight-btag", new Pose3d(Units.inchesToMeters(2.670592), Units.inchesToMeters(-3.0), Units.inchesToMeters(25.283), new Rotation3d(0.0, Units.degreesToRadians(24.0), Units.degreesToRadians(180.0)))
    );

    public static final String ALGAE_CAMERA_NAME = "limelight-balgae";
    public static final int[] BLACKLISTED_APRILTAGS = {};

    public static final double SPHERE_TOLERANCE = 0.5;

    public static final Rotation2d ALGAE_CAMERA_PITCH = Rotation2d.fromDegrees(-24); //CHANGE (DEGREES)
    // x is forward, y is left, z is up
    public static final Translation3d ALGAE_CAMERA_POSITION = new Translation3d(Units.inchesToMeters(13.0), Units.inchesToMeters(0.0), Units.inchesToMeters(22.5));

    public static final Rotation2d FOV_HEIGHT = Rotation2d.fromDegrees(48.9); // Degrees
    public static final Rotation2d FOV_WIDTH = Rotation2d.fromDegrees(62.5); // Degrees
    public static final double ALGAE_CAMERA_HEIGHT_PIXELS = 960;

    public static final double MAX_DETECTION_RANGE = 19.30;
  }

  public static final class SWERVE {
    public static final Slot0Configs DRIVE_MOTOR_GAINS = new Slot0Configs()
      .withKP(0.01)
      .withKD(0.01)
      .withKI(0.1)
      .withKV(0.117)
      .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    public static final Slot0Configs STEER_MOTOR_GAINS = new Slot0Configs()
      .withKP(20)
      .withKI(1)
      .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    public static final DriveGains DRIVE_GAINS = new DriveGains(
      new PIDConstants(5.0, 1.0, 0),
      new PIDConstants(5.0, 1.0, 0)
    );

    public static final Chassis CHASSIS = new Chassis(
      Inches.of(30),
      Inches.of(30),
      Inches.of(24.75),
      Inches.of(24.75),
      Pounds.of(135)
    );

    public static final SwerveConfig.Module[] MODULE_CONFIGS = {
      new SwerveConfig.Module(10, 20, 30, Radians.of(0.192)),
      new SwerveConfig.Module(11, 21, 31, Radians.of(-1.911)),
      new SwerveConfig.Module(12, 22, 32, Radians.of(1.555)),
      new SwerveConfig.Module(13, 23, 33, Radians.of(-0.019)),
      new SwerveConfig.Module(14, 24, 34, Degrees.of(0)),
      new SwerveConfig.Module(15, 25, 35, Degrees.of(0)),
      new SwerveConfig.Module(16, 26, 36, Degrees.of(0)),
      new SwerveConfig.Module(17, 27, 37, Degrees.of(0)),
      new SwerveConfig.Module(18, 28, 38, Degrees.of(0)),
    };

    public static final SwerveConfig.Module[] SELECTED_MODULE_CONFIGS = {
      MODULE_CONFIGS[0], // front-left
      MODULE_CONFIGS[3], // front-right
      MODULE_CONFIGS[1], // back-left
      MODULE_CONFIGS[2]  // back-right
    };

    public static final SwerveConfig CONFIG = new SwerveConfig(
      CHASSIS,
      Gearing.MK4I_L2_PLUS,
      SELECTED_MODULE_CONFIGS,
      new Motor(DCMotor.getKrakenX60(1), DRIVE_MOTOR_GAINS, Amps.of(60)),
      new Motor(DCMotor.getKrakenX60(1), STEER_MOTOR_GAINS, Amps.of(60)),
      Wheel.COLSON,
      DRIVE_GAINS
    );
  }

  public static final class CAN {
    // In order of: front left, front right, back left, back right, where the battery is in the back
    public static final int PDH = 1;
    public static final int SHOOTER_WHEELS_TOP = 19;
    public static final int SHOOTER_WHEELS_BOTTOM = 26;
    public static final int SHOOTER_PIVOT = 18;
    public static final int SHOOTER_FEED = 20;
    public static final int TRANSFER_OUT = 24;
    public static final int TRANSFER_IN = 22;
    public static final int INTAKE = 29; 
    public static final int HANG = 0; // UPDATE 
    public static final int ELEVATOR_ENCODER = 0; // UPDATE
    
  }

  public static final class DIO {
    public static final int AMP_PIVOT = 1;
    public static final int MANIPULATOR_PIVOT = 2;
    public static final int BEAM_BREAK = 3;
    public static final int HANG_ENCODER = 0; //UPDATE
    public static final int ELEVATOR_ENCODER = 4;
    
  }

  public static final class NEO {
    public static final DCMotor STATS = new DCMotor(12.0, 3.0, 160.0, 2.065, Units.rotationsPerMinuteToRadiansPerSecond(5820), 1);

    public static final double SAFE_TEMPERATURE = 60;
    public static final int SAFE_STALL_CURRENT = 40;
    public static final int SAFE_FREE_CURRENT = 60;
    public static final double SAFE_RAMP_RATE = 0.1;

    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }
  public static final class NEO550 {
    public static final DCMotor STATS = DCMotor.getNeo550(1);
    public static final double SAFE_TEMPERATURE = 60;
    public static final int SAFE_STALL_CURRENT = 10;
    public static final double SAFE_RAMP_RATE = 0.1;
    // 
    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }

  public static final class KRAKEN {
    public static final DCMotor STATS = new DCMotor(
      12.0, 
      7.09, 
      366.0, 
      2.0,
      Units.rotationsPerMinuteToRadiansPerSecond(6000), 
      1
    );
    public static final int SAFE_STALL_CURRENT = 120;
    public static final int SAFE_FREE_CURRENT = 70;
    public static final double SAFE_RAMP_RATE = 0.1;

    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }

  // public static final class AUTONOMOUS {
  //   public static final double ACCELERATION_REDUCTION = ((SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION * SWERVE_DRIVE.ROBOT_MASS + ((SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA * SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION) / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS)) / (9.80 * SWERVE_DRIVE.ROBOT_MASS * SWERVE_DRIVE.FRICTION_COEFFICIENT));

  //   public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
  //       new PathConstraints(
  //         SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
  //         SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION / ACCELERATION_REDUCTION,
  //         SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY,
  //         SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION / ACCELERATION_REDUCTION
  //       );
  //   }
  // }

  public static final class ELEVATOR {
    public static final double ENCODER_CONVERSION_FACTOR = 1.0; // CALCULATE
    public static final double ELEVATOR_MAX_HEIGHT = 40; // Placeholder, in inches
    public static final double ELEVATOR_MIN_HEIGHT = 2; // Placeholder, in inches
  }
  public static final class SHOOTER_FEED {
    public static final double GEARING = 1.0;
    public static final double FREE_TORQUE = 1.0; // TODO
    public static final double RADIUS = Units.inchesToMeters(1.0);
  }
  public static final class SHOOTER_WHEELS {
    public static final double GEARBOX_STEP_UP = 2.0;
    public static final double ENCODER_CONVERSION_FACTOR = 2.0 * Math.PI * GEARBOX_STEP_UP;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double WHEEL_MOI = 0.00018540712;
    public static final double TOTAL_MOI = WHEEL_MOI * 12.0;
    public static final double PROJECTILE_MASS = Units.lbsToKilograms(0.5);
    public static final double COMPRESSION = Units.inchesToMeters(0.5);
    public static final double SPEED_PRECISION = Units.rotationsPerMinuteToRadiansPerSecond(10);
    public static final double TOP_EXIT_VELOCITY = 12.5;
    public static final double MAX_EXIT_VELOCITY = 13.0;
    public static final double NOTE_LOG_BASE = 1.0082;
    public static final double NOTE_LOG_OFFSET = 504.213;
    public static final double MAX_WHEEL_SPEED = NEO.STATS.freeSpeedRadPerSec * SHOOTER_WHEELS.GEARBOX_STEP_UP;

    // x is front-to-back
    // y is left-to-right
    // z is top-to-bottom
    
    public static final class PROFILE {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.0; // volts per rad/s
      public static final double kV = (12.0 / (NEO.STATS.freeSpeedRadPerSec * GEARBOX_STEP_UP)) / 0.8; // volts per rad/s
      public static final double kA = 0.0; // volts per rad/s^2
    }
  }

  public static final class MANIPULATOR_PIVOT {
    public static final double GEARING = 15.0 * (78.0 / 20.0) * (200.0 / 19.0);
    public static final double ROTATION_DELAY = 0.3; // seconds
    public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.25);
    public static final Rotation2d ANGLE_PRECISION = Rotation2d.fromDegrees(0.25);
    public static final Rotation2d HEADING_PRECISION = Rotation2d.fromDegrees(0.25);
    public static final Translation3d POSITION = new Translation3d(Units.inchesToMeters(3.33), 0.0, Units.inchesToMeters(12.1));
    public static final double ABSOLUTE_POSITION_OFFSET = Units.degreesToRotations(-140.375 - 93.15 + 27); //  - [ rawAbsolutePosition from logs ] - 93.15 + [ the angle measured from the front plate of shooter ]
    public static final Rotation2d NOTE_ROTATION_OFFSET = Rotation2d.fromDegrees(-1.25); // Theoretically 3.1480961
    public static final double SHOOTER_LENGTH = Units.inchesToMeters(15.023);
    
    public static final class PROFILE {
      public static final double kP = 15.0;
      public static final double kS = 0.2;
      public static final double MAX_ACCELERATION = 30.0; // rad/s^2
    }
  }
  
  public static final class INTAKE {
    public static final double GEARING = (38.0 / 16.0) * (15.0 / 18.0);
    public static final double RADIUS = Units.inchesToMeters(0.5);
    public static final double MAX_LINEAR_SPEED = NEO.STATS.freeSpeedRadPerSec / GEARING * RADIUS;
  }

  public static final class HANG {
    public static final Double EXTEND_HEIGHT = 0.0; //CHANGE
    public static final Double RETRACT_HEIGHT = 0.0; //CHANGE
  }
  // LED
  public static final class LED {
    public static final int SIDE_STRIP_HEIGHT = 58; // Number of LEDs on side strip
  }
}
