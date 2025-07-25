// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;

import java.io.InputStream;
import java.util.Properties;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Field;
import frc.robot.Constants.ReefPositioning;
import frc.robot.Constants.Constants.AUTO;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.SWERVE;
import frc.robot.auto.choreo.AutonomousV3;
import frc.robot.auto.choreo.AutonomousV3.Side;
import frc.robot.auto.pipeline.AutoGeneration;
import frc.robot.auto.utils.AutoCommands;
import frc.robot.auto.utils.AutoPaths;
import frc.robot.commands.PieceCombos;
import frc.robot.commands.SafeSubsystems;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Algae;
import frc.robot.util.CachedRobotState;
import frc.robot.util.RobotEvent;
import frc.robot.util.software.Dashboard.AutonChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer instance;
  public static RobotEvent disabledPeriodic = new RobotEvent();

  /**
   * Get the RobotContainer instance (for testing or competition only!)
   *
   * @return
   */
  public static RobotContainer getInstance() {
    return instance;
  }

  // The robot's subsystems and commands
  public final SwerveDrive swerveDrive;
  public final Manipulator manipulator;
  public final Elevator elevator;
  public final Hang hang;
  public final AutoCommands autov2;
  public final AutonomousV3 autov3;
  public final Algae algaeDetector;
  private final LEDs ledStrip;
  public final PieceCombos pieceCombos;
  public final SafeSubsystems safeties;
  // public final ManipulatorSafeties manipulatorSafeties;
  // private final CollisionDetector collisionDetector;

  private static PowerDistribution PDH = new PowerDistribution(CAN.PDH, ModuleType.kRev);

  private final AutoGeneration autoGen;

  // private SwerveModuleTest swerveModuleTest = new SwerveModuleTest();

  // private SteerModuleTest steerModuleTest = new SteerModuleTest();

  // private DriveModuleTest test;

  // private SwerveModuleTest swerveModuleTest = new SwerveModuleTest();

  SwerveModule module;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    instance = this;

    var log = DataLogManager.getLog();
    DriverStation.startDataLog(log, true);
    logGitProperties(log);
    // Logger.autoLog("PDH", PDH);

    CachedRobotState.init();

    AutonChooser.init();

    LiveWindow.disableAllTelemetry();

    DriverStation.silenceJoystickConnectionWarning(true);

    StatusChecks.Category statusChecks = StatusChecks.under("General");

    statusChecks.add("FMS Attached", () -> DriverStation.isFMSAttached());
    statusChecks.add("DS Attached", () -> DriverStation.isDSAttached());
    statusChecks.add("Joystick 0", () -> DriverStation.isJoystickConnected(0));
    statusChecks.add("Joystick 1", () -> DriverStation.isJoystickConnected(1));
    statusChecks.add("Battery Voltage", () -> RobotController.getBatteryVoltage() > 12.0);
    // statusChecks.add("RSL", () -> RobotController.getRSLState());
    statusChecks.add("Loop Time", () -> Robot.getComputeTime() <= 0.02);
    statusChecks.add("3V3 Enabled", () -> RobotController.getEnabled3V3());
    statusChecks.add("5V Enabled", () -> RobotController.getEnabled5V());
    statusChecks.add("6V Enabled", () -> RobotController.getEnabled6V());
    statusChecks.add("Sys Time Valid", () -> RobotController.isSystemTimeValid());

    Logger.logEnabledSystems();

    swerveDrive = new SwerveDrive(SWERVE.CONFIG);
    ledStrip = new LEDs();

    manipulator = new Manipulator();
    elevator = Elevator.create();
    safeties = new SafeSubsystems(elevator, manipulator);
    pieceCombos = new PieceCombos(elevator, manipulator, safeties);
    autov2 = new AutoCommands(swerveDrive, manipulator, elevator, pieceCombos);
    autov3 = new AutonomousV3(swerveDrive, manipulator, elevator, pieceCombos);
    algaeDetector = new Algae();
    hang = Hang.create();
    // // collisionDetector = new CollisionDetector();

    // System.out.println(swerveDrive);

    // // Configure the trigger bindings
    Controls.configureBindings(swerveDrive, elevator, manipulator, hang, autov2, autov3, pieceCombos);

    autoGen =
        new AutoGeneration(
            autov2,
            AUTO.SLEEP_TIME,
            AUTO.WORK_TIME,
            () ->
                AutoPaths.PlanParameters.fromAutoChooser(
                    manipulator.grabber.hasCoral(), swerveDrive.getEstimatedPose()));

    // module = new SwerveModule();
    NetworkTableEntry refreshButtonEntry =
        NetworkTableInstance.getDefault().getTable("StatusChecks").getEntry("refreshButton");

    statusChecks.timestampAdd("timerChecker", () -> Timer.getFPGATimestamp());

    refreshButtonEntry.setBoolean(false);

    // System.out.println(ReefPositioning.getCoralPlacePose(3));

    // module.configureModule(Constants.SWERVE.CONFIG, Corner.FRONT_LEFT);

    // AprilTags.printConfig(Constants.LIMELIGHT.APRILTAG_CAMERA_POSES);

    // Pathfinding.ensureInitialized();

    // swerveModuleTest = new SwerveModuleTest();

    // new Talon10Test();

    // steerModuleTest = new SteerModuleTest();

    // test = new DriveModuleTest();

    // ChassisSpeeds testSpeeds = new ChassisSpeeds(0, 0, 1);

    // Logger.log("conversionTest/speeds", testSpeeds);
    // Logger.log("conversionTest/states",
    // KinematicsUtils.kinematicsFromChassis(Constants.SWERVE.CHASSIS).toSwerveModuleStates(testSpeeds));

    Logger.start(Milliseconds.of(20));
  }

  public Command getAutonomousCommand() {
    // AUTO ROUTINES - Uncomment the one you want to run

    // 1. Start in the middle of the field, then score 3 coral on the right side.
    // return autov3.createSideAutonomous(Side.RIGHT, true);

    // 2. Start in the middle of the field, then score 3 coral on the left side.
    // return autov3.createSideAutonomous(Side.LEFT, true);

    // 3. Start on the right side of the field, then score 3 coral on the right side.
    // return autov3.createSideAutonomous(Side.RIGHT, false);

    // 4. Start on the left side of the field, then score 3 coral on the left side.
    // return autov3.createSideAutonomous(Side.LEFT, false);

    // 5. Start in the middle field, score preloaded coral on the back face then
    //    throw the algae on the back face into the barge.
    // return autov3.createMiddleAutonomous();

    // 6. Drive foreward until astop
    // return swerveDrive.drive(new ChassisSpeeds(0.5, 0, 0));

    // 7. Do nothing
    return Commands.none();

    // 8. Old autonomous
    // return autoGen.getCommand();

    ////////////////////////////////////////////////////////////////////////////
    // OTHER COMMANDS - Do not use in matches

    // 9. Calibrate wheel size for odometry
    // return swerveDrive.calibrateWheelSize();
  }

  public static double getVoltage() {
    return RobotController.getBatteryVoltage();
  }

  public static double getTotalCurrent() {
    return PDH.getTotalCurrent();
  }

  public static PowerDistribution getPDH() {
    return PDH;
  }

  public void latePeriodic() {
    swerveDrive.latePeriodic(); // TODO: Uncomment before use

    // Pose2d[] poses;

    // Translation2d algae =
    //     Algae.getAlgaePosition("limelight-algae", swerveDrive, LIMELIGHT.ALGAE_CAMERA_POSITION);

    // if (algae != null) {
    //   poses = new Pose2d[] {new Pose2d(algae, new Rotation2d())};
    // } else {
    //   poses = new Pose2d[0];
    // }

    // Logger.getField().getObject("Algae").setPoses(poses);
  }

  public void disabledPeriodic() {
    // System.out.println(new Translation2d(0.0,0.0));
    // System.out.println((Algae.getAlgaePosition(LIMELIGHT.ALGAE_CAMERA_NAME, swerveDrive,
    // LIMELIGHT.ALGAE_CAMERA_POSITION)));

    // autoGen.work();

    disabledPeriodic.run();
  }

  public void disabledInit() {}

  public void testInit() {
    // module.calibrateSteerMotor(RobotController.getMeasureBatteryVoltage(),
    // Amps.of(60)).schedule();

    // Commands.sequence(manipulator.pivot.safe(), elevator.rezeroAtBottom()).schedule();

    // Command checks = new PrematchChecks(swerveDrive, elevator, manipulator, null);
    // checks.schedule();

    // elevator.rezeroAtBottom().schedule();
    // LEDs.setStateCommand(LEDs.State.ENABLED).schedule();;

    // swerveDrive.getModules()[0].calibrateSteerMotor(Amps.of(80)).schedule();
  }

  private final void logGitProperties(DataLog log) {
    // Load git properties from classpath
    Properties gitProps = new Properties();
    try (InputStream is = getClass().getClassLoader().getResourceAsStream("git.properties")) {
      if (is != null) {
        gitProps.load(is);

        // Log all git properties
        gitProps.forEach(
            (key, value) -> {
              String propertyPath = "/Metadata/" + key.toString();
              StringLogEntry entry = new StringLogEntry(log, propertyPath);
              entry.append(value.toString());

              // Also print to console for debugging
              System.out.println(key + ": " + value);
            });
      } else {
        System.err.println("git.properties not found in classpath");
      }
    } catch (Exception e) {
      System.err.println("Failed to load git properties: " + e.getMessage());
    }
  }
}
