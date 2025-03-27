// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;

import java.io.InputStream;
import java.util.Properties;

import com.team6962.lib.prepath.CustomLocalADStar;
import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.Constants.AUTO;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.SWERVE;
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
  public final AutoCommands autonomous;
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
    autonomous = new AutoCommands(swerveDrive, manipulator, elevator, pieceCombos);
    algaeDetector = new Algae();
    hang = Hang.create();
    // // collisionDetector = new CollisionDetector();

    // System.out.println(swerveDrive);

    // // Configure the trigger bindings
    Controls.configureBindings(swerveDrive, elevator, manipulator, hang, autonomous, pieceCombos);

    autoGen =
        new AutoGeneration(
            autonomous,
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
    // return autonomous.createDemoAutonomousCommand();
    // return Commands.defer(() -> {
    //   // System.out.println("Generating auto command");

    //   // return autoGen.generate();

    //   GeneratedAuto auto = new GeneratedAuto(autonomous,
    // AutoParams.get(swerveDrive.getEstimatedPose(), manipulator.coral.hasGamePiece()));
    //   auto.setup();

    //   return auto.getCommand();
    // }, Set.of(swerveDrive, elevator, manipulator.coral, manipulator.pivot));

    // return autonomous.createAutonomousCommand();

    Command auto = autoGen.getCommand();

    return auto;

    // return swerveDrive.pathfindToPrecomputed(new Pose2d(1, 1, Rotation2d.fromDegrees(0)), new Pose2d(6.5, 6.5, Rotation2d.fromDegrees(70)));
    
    // return swerveDrive.pathfindBetweenWaypoints(
    //   new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //   new Pose2d(
    //     Math.random() * 5, Math.random() * 3,
    //     Rotation2d.fromRotations(Math.random())
    //   ),
    //   Rotation2d.fromDegrees(0),
    //   Rotation2d.fromRotations(Math.random()),
    //   MetersPerSecond.of(0),
    //   MetersPerSecond.of(1)
    // );

    // return swerveDrive.drive(new ChassisSpeeds(0, 0, 100000));

    // return swerveDrive.getModules()[0].calibrateSteerMotor(Amps.of(60));

    // return Commands.sequence(
    //   // elevator.calibrate()
    //   manipulator.pivot.calibrate()
    // );

    // return Commands.sequence(
    // elevator.calibrate()
    // manipulator.pivot.calibrate());
    // return hang.stow();
    // return Commands.run(() -> {});
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

    swerveDrive.getModules()[0].calibrateSteerMotor(Amps.of(80)).schedule();
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
