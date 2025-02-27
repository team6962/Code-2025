// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class TEAM_COLOR {
    public static final Supplier<Boolean> IS_BLUE_TEAM =
        () ->
            !(DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
  }

  // ENABLED SYSTEMS
  public static final class ENABLED_SYSTEMS {
    public static final boolean DRIVE = true;
    public static final boolean DASHBOARD = true;
    public static final boolean INTAKE = false;
    public static final boolean HANG = false;
    public static final boolean MANIPULATOR = true;
    public static final boolean ELEVATOR = true;
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
    public static final Distance ALGAE_DIAMETER = Inches.of(16.25); // meters
    public static final boolean SIMULATE_STATIC_ALGAE = false;
  }

  // LIMELIGHT
  // Exposure: 750
  // Sensor Gain: 10
  public static final class LIMELIGHT {
    // x is front-to-back
    // y is left-to-right
    // z it top-to-bottom
    public static final Map<String, Pose3d> APRILTAG_CAMERA_POSES =
        Map.of(
            "limelight-ftag",
                new Pose3d(
                    Units.inchesToMeters(14.5),
                    Units.inchesToMeters(0.0),
                    Units.inchesToMeters(7.5),
                    new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(19.0), 0.0)),
            "limelight-btag",
                new Pose3d(
                    Units.inchesToMeters(0.0),
                    Units.inchesToMeters(0.0),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0))));

    public static final String ALGAE_CAMERA_NAME = "limelight-algae";
    public static final int[] BLACKLISTED_APRILTAGS = {};

    public static final double SPHERE_TOLERANCE = 0.5;

    public static final Rotation2d ALGAE_CAMERA_PITCH =
        Rotation2d.fromDegrees(-24); // CHANGE (DEGREES)
    // x is forward, y is left, z is up
    public static final Translation3d ALGAE_CAMERA_POSITION =
        new Translation3d(
            Units.inchesToMeters(3.0), Units.inchesToMeters(0.0), Units.inchesToMeters(32.5));

    public static final Rotation2d FOV_HEIGHT = Rotation2d.fromDegrees(48.9); // Degrees
    public static final Rotation2d FOV_WIDTH = Rotation2d.fromDegrees(62.5); // Degrees
    public static final double ALGAE_CAMERA_HEIGHT_PIXELS = 960;

    public static final Distance MAX_DETECTION_RANGE =
        Meters.of(19.30); // Max distance an algae can be while being on the field
  }

  // Competition: 5476
  public static final class SWERVE {
    public static final Angle ALL_OFFSET = Degrees.of(135);

    public static final SwerveConfig.Module[] MODULE_CONFIGS = {
      new SwerveConfig.Module(10, 20, 30, Radians.of(0.192)),
      new SwerveConfig.Module(11, 21, 31, Radians.of(-1.911)),
      new SwerveConfig.Module(12, 22, 32, Radians.of(1.555)),
      new SwerveConfig.Module(13, 23, 33, Radians.of(-0.019)),
      new SwerveConfig.Module(
          14, 24, 34, Radians.of(-2.439 - Math.PI / 4).plus(ALL_OFFSET)), // -2.439
      new SwerveConfig.Module(
          15, 25, 35, Radians.of(-0.440 + Math.PI / 2 + Math.PI / 4).plus(ALL_OFFSET)), // -0.440
      new SwerveConfig.Module(
          16,
          26,
          36,
          Radians.of(-1.842 - Math.PI / 2 - 3.0 / 4.0 * Math.PI).plus(ALL_OFFSET)), // -1.842
      new SwerveConfig.Module(
          17,
          27,
          37,
          Radians.of(-1.049 - Math.PI + 3.0 / 4.0 * Math.PI).plus(ALL_OFFSET)), // -1.049
      new SwerveConfig.Module(18, 28, 38, Degrees.of(0)),
    };

    public static final SwerveConfig CONFIG = SwerveConstants.get();
  }

  public static final class SWERVE_DRIVE {
    public static final double TELEOPERATED_FINE_TUNE_DRIVE_POWER =
        0.1; // Percent driving power when using d-pad
    public static final double TELEOPERATED_DRIVE_POWER = 0.5; // Percent driving power
    public static final double TELEOPERATED_BOOST_POWER =
        1.0; // Percent power when using the triggers
    public static final double TELEOPERATED_ROTATE_POWER = 0.1; // Percent rotating power
    public static final double TELEOPERATED_ROTATE_BOOST_POWER = 0.2; // Percent rotating power
  }

  public static final class CAN {
    // In order of: front left, front right, back left, back right, where the battery is in the back
    public static final int PDH = 1;
    public static final int INTAKE_WHEELS = 28;
    public static final int INTAKE_PIVOT = 29;
    public static final int HANG = 34;
    public static final int ELEVATOR_LEFT = 2;
    public static final int ELEVATOR_RIGHT = 3;
    public static final int MANIPULATOR_PIVOT = 4;
    public static final int MANIPULATOR_ALGAE_RIGHT = 5;
    public static final int MANIPULATOR_ALGAE_LEFT = 6;
    public static final int MANIPULATOR_CORAL = 7;
  }

  public static final class DIO {
    public static final int HANG_ENCODER = 0;
    public static final int MANIPULATOR_ENCODER = 2;
    public static final int CORAL_BEAM_BREAK = 3;
    public static final int ELEVATOR_CEIL_LIMIT = 4;
    public static final int ELEVATOR_FLOOR_LIMIT = 5;
  }

  public static final class PWM {
    public static final int LEDS = 9;
  }
  public static final class NEO {
    public static final DCMotor STATS =
        new DCMotor(12.0, 3.0, 160.0, 2.065, Units.rotationsPerMinuteToRadiansPerSecond(5820), 1);

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
    public static final DCMotor STATS =
        new DCMotor(12.0, 7.09, 366.0, 2.0, Units.rotationsPerMinuteToRadiansPerSecond(6000), 1);
    public static final int SAFE_STALL_CURRENT = 120;
    public static final int SAFE_FREE_CURRENT = 70;
    public static final double SAFE_RAMP_RATE = 0.1;

    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }

  // public static final class AUTONOMOUS {
  //   public static final double ACCELERATION_REDUCTION =
  // ((SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION * SWERVE_DRIVE.ROBOT_MASS +
  // ((SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA * SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION) /
  // SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS)) / (9.80 * SWERVE_DRIVE.ROBOT_MASS *
  // SWERVE_DRIVE.FRICTION_COEFFICIENT));

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
    public static final double GEARING =
        (3.0 / 1.0)
            * (3.0 / 1.0)
            * (5.0 / 1.0)
            * (3.0 / 2.0)
            / 2.0; // CALCULATE LAST VALUE FOR STAGES IN THE ELEVATOR
    public static final Distance CYCLE_HEIGHT = Inches.of(2.15 * Math.PI); // CALCULATE
    public static final Distance Bhobe_HEIGHT = Inches.of(1);

    public static final NavigableMap<Double, AngleRange> HEIGHT_TO_ANGLE_MAP = new TreeMap<>();

    static {
      HEIGHT_TO_ANGLE_MAP.put(0.0, new AngleRange(Degrees.of(0.0), Degrees.of(45.0)));
      HEIGHT_TO_ANGLE_MAP.put(1.0, new AngleRange(Degrees.of(10.0), Degrees.of(40.0)));
      HEIGHT_TO_ANGLE_MAP.put(2.0, new AngleRange(Degrees.of(20.0), Degrees.of(35.0)));
      HEIGHT_TO_ANGLE_MAP.put(3.0, new AngleRange(Degrees.of(30.0), Degrees.of(30.0)));
      HEIGHT_TO_ANGLE_MAP.put(4.0, new AngleRange(Degrees.of(40.0), Degrees.of(25.0)));
    }

    public static final class PROFILE {
      public static final double kP = 20.0;
      public static final double kS = 1.0;
    }

    // HEIGHT IS MEASURED FROM THE GROUND TO THE TOP OF THE ELEVATOR
    public static final Distance BASE_HEIGHT = Inches.of(35.5);
    public static final Distance MAX_HEIGHT = Inches.of(83.6);
    public static final Distance MIN_HEIGHT = BASE_HEIGHT;
    public static final Distance STOW_HEIGHT = BASE_HEIGHT;
    public static final Distance MAX_UNLIMITED_HEIGHT = Inches.of(41.0); // AVERAGE

    public static final class CORAL {
      public static final Distance L1_HEIGHT = Inches.of(39.5);
      public static final Distance L2_HEIGHT = Inches.of(48.4);
      public static final Distance L3_HEIGHT = Inches.of(59.2);
      public static final Distance L4_HEIGHT = Inches.of(80.2);
      public static final Distance INTAKE_HEIGHT = Inches.of(58.5);
    }

    public static final class ALGAE {
      public static final Distance L2_HEIGHT = Inches.of(53.0);
      public static final Distance L3_HEIGHT = Inches.of(64.0);
      public static final Distance BARGE_HEIGHT = MAX_HEIGHT;
      public static final Distance GROUND_HEIGHT = MIN_HEIGHT;
      public static final Distance PROCESSOR_HEIGHT = MIN_HEIGHT;
    }
  }

  public static final class MANIPULATOR_PIVOT {
    public static final double GEARING = 5 * 5 * 5;
    public static final double ROTATION_DELAY = 0.3; // seconds
    public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.25);
    public static final Angle ABSOLUTE_POSITION_OFFSET = Rotations.of(0.512);
    public static final Angle CENTER_OF_MASS_OFFSET = Degrees.of(69.134); // CALCULATED FROM CAD

    public static final class PROFILE {
      public static final double kP =  4.0; //3.75;
      public static final double kI = 0.0;
      public static final double kD = 0.0; // 10.0; // 0.1;
      public static final double kS = 0.15;
      public static final double MAX_ACCELERATION = 30.0; // rad/s^2
    }

    public static final Angle MAX_ANGLE = Degrees.of(36.0); // RESET TO 40.0
    public static final Angle MIN_ANGLE = Degrees.of(-135.5);

    public static final Angle STOW_ANGLE = Degrees.of(35.0);
    public static final Angle SAFE_ANGLE = Degrees.of(-34.5);

    public static final Angle SAFE_MIN_ANGLE = Degrees.of(-45);
    public static final Angle SAFE_MAX_ANGLE = Degrees.of(-18.0);

    public static final Angle PID_MIN_ANGLE = Degrees.of(-90.0);
    public static final Angle PID_MID_ANGLE = Degrees.of(-45.0);
    public static final Angle PID_MAX_ANGLE = Degrees.of(0.0);
    public static final Angle TOLERANCE = Degrees.of(2.0);

    public static final NavigableMap<Distance, Angle> MIN_ANGLES;
    static {
      MIN_ANGLES = new TreeMap<>();
      MIN_ANGLES.put(Inches.of(45), SAFE_MIN_ANGLE);
      MIN_ANGLES.put(Inches.of(55), Degrees.of(-90.0));
      MIN_ANGLES.put(Inches.of(Double.POSITIVE_INFINITY), MIN_ANGLE);      
    }

    public static final NavigableMap<Distance, Angle> MAX_ANGLES;
    static {
      MAX_ANGLES = new TreeMap<>();
      MAX_ANGLES.put(Inches.of(39), MAX_ANGLE);
      MAX_ANGLES.put(Inches.of(43), Degrees.of(22.0));
      MAX_ANGLES.put(Inches.of(48), Degrees.of(-6.0));
      MAX_ANGLES.put(Inches.of(81.5), SAFE_MAX_ANGLE);
      MAX_ANGLES.put(Inches.of(Double.POSITIVE_INFINITY), Degrees.of(22.0));      
    }

    public static final class CORAL {
      public static final Angle L1_ANGLE = Degrees.of(0.0);
      public static final Angle L23_ANGLE = Degrees.of(-34.5);
      public static final Angle L4_ANGLE = Degrees.of(-56.5);
      public static final Angle INTAKE_ANGLE = Degrees.of(-135.4);
    }

    public static final class ALGAE {
      public static final Angle BARGE_ANGLE = Degrees.of(21.90);
      public static final Angle REEF_ANGLE = Degrees.of(-23.0);
      public static final Angle GROUND_ANGLE = Degrees.of(-35.3);
      public static final Angle PROCESSOR_ANGLE = Degrees.of(0.0);
    }
  }

  public static final class MANIPULATOR {
    public static final boolean INVERT_ALGAE_LEFT = true;
    public static final boolean INVERT_ALGAE_RIGHT = true;

    public static final Current ALGAE_DETECT_CURRENT = Amps.of(15);
    public static final Time ALGAE_GRIP_CHECK_TIME = Seconds.of(0.25);
    public static final Time ALGAE_GRIP_CHECK_RATE = Seconds.of(5.0);
    public static final boolean ALGAE_GRIP_CHECK_ENABLED = true;

    public static final double ALGAE_OUT_SPEED = -1.0;
    public static final double ALGAE_IN_SPEED = 0.5;
    public static final double ALGAE_HOLD_SPEED = 0.1;

    public static final double ALGAE_GRIP_CHECK_SPEED = 0.1;
    public static final double CORAL_OUT_SPEED = 1.0;
    public static final double CORAL_IN_SPEED = 0.2;
    public static final double CORAL_HOLD_SPEED = -0.0;
  }

  public static final class INTAKE {
    public static final double GEARING = 10.0;
    public static final Distance LENGTH = Inches.of(16.5);
    public static final Mass MASS = Kilograms.of(1.85);
    public static final PIDConstants PID = new PIDConstants(10.0, 0.0, 0.0);
    public static final double IN_POWER = 0.5;
    public static final double OUT_POWER = 0.5; // Placeholder value

    public static final Angle PIVOT_DOWN = Degrees.of(0.0);
    public static final Angle PIVOT_UP = Degrees.of(90.0);

    public static final double TO_SHOOTER_POWER = 0.3;
    public static final double SLOW_OUT_POWER = 0.4;
  }

  public static final class HANG_PIVOT {
    public static final double GEARING = (9.0 / 1.0) * (9.0 / 1.0) * (3.0 / 1.0) * (26.0 / 12.0);
    public static final Angle ENCODER_OFFSET = Rotations.of(0.085);

    public static final Angle MAX_ANGLE = Degrees.of(140.0);
    public static final Angle MIN_ANGLE = Degrees.of(-40.0);
    public static final Angle STOW_ANGLE = Degrees.of(0.0);
    public static final Angle HANG_ANGLE = Degrees.of(-40.0);

    public static final class PROFILE {
      public static final double kP = 0.5;
      public static final double kS = 0.5; // volts per rad/s
      public static final double kD = 0.0;
      public static final double kI = 0.0;
    }
  }

  public static final class HANG { // Adjust these as needed
    public static final double CLIMB_POWER = 0.5;
    public static final double REVERSE_POWER = 0.5;
  }

  // LED
  public static final class LED {
    public static final int SIDE_STRIP_HEIGHT = 58; // Number of LEDs on side strip
  }

  public static final class VOLTAGE_LADDER {
    public static final double SWERVE_DRIVE = 7.0;
    public static final double ELEVATOR = 7.5;
    public static final double MANIPULATOR = 8.0;
    public static final double INTAKE = 8.5;
    public static final double HANG = 9.0;
  }

  public static final class TIMING {
    public static final Time MATCH_LENGTH = MeasureMath.time(2, 30);
    public static final Time AUTO_LENGTH = MeasureMath.time(0, 15);
    public static final Time ENDGAME_LENGTH = MeasureMath.time(0, 20);
    public static final Time ENDGAME_START = MATCH_LENGTH.minus(ENDGAME_LENGTH);
  }

  public static class AngleRange {
    private final Angle minAngle;
    private final Angle maxAngle;

    public AngleRange(Angle minAngle, Angle maxAngle) {
      this.minAngle = minAngle;
      this.maxAngle = maxAngle;
    }

    public Angle getMinAngle() {
      return minAngle;
    }

    public Angle getMaxAngle() {
      return maxAngle;
    }
  }
}
