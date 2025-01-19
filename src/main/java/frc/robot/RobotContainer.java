// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;
import com.team6962.lib.test.KinematicsTest;
import com.team6962.lib.test.SwerveModuleTest;
import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.vision.AprilTags;
import frc.robot.util.software.Dashboard.AutonChooser;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer instance;

  /**
   * Get the RobotContainer instance (for testing or competition only!)
   * @return
   */
  public static RobotContainer getInstance() {
    return instance;
  }

  // The robot's subsystems and commands
  public final SwerveDrive swerveDrive;
  public final RobotStateController stateController;
  public final LEDs ledStrip;
  // private final CollisionDetector collisionDetector;

  private static PowerDistribution PDH = new PowerDistribution(CAN.PDH, ModuleType.kRev);

  private SwerveModuleTest swerveModuleTest = new SwerveModuleTest();

  // private SteerModuleTest steerModuleTest = new SteerModuleTest();

  private KinematicsTest kinematicsTest;

  // private SwerveModuleTest swerveModuleTest = new SwerveModuleTest();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    instance = this;

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    // Logger.autoLog("PDH", PDH);

    Logger.start(Milliseconds.of(20));
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

    swerveDrive = new SwerveDrive(Constants.SWERVE.CONFIG);
    stateController = new RobotStateController(swerveDrive);
    ledStrip = new LEDs(stateController);
    // // collisionDetector = new CollisionDetector();

    // System.out.println(swerveDrive);
    
    // // Configure the trigger bindings
    Controls.configureBindings(stateController, swerveDrive);

    // AprilTags.printConfig(Constants.LIMELIGHT.APRILTAG_CAMERA_POSES);

    Pathfinding.ensureInitialized();

    swerveModuleTest = new SwerveModuleTest();

    // steerModuleTest = new SteerModuleTest();

    // kinematicsTest = new KinematicsTest(Constants.SWERVE.CONFIG);

    // ChassisSpeeds testSpeeds = new ChassisSpeeds(0, 0, 1);

    // Logger.log("conversionTest/speeds", testSpeeds);
    // Logger.log("conversionTest/states", KinematicsUtils.kinematicsFromChassis(Constants.SWERVE.CHASSIS).toSwerveModuleStates(testSpeeds));
  }

  public Command getAutonomousCommand() {
    // return new Autonomous(stateController, swerveDrive, AutonChooser.getNotes());
    // return Commands.run(() -> {});
    return Commands.run(() -> {});//swerveDrive.park();
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
    swerveDrive.latePeriodic();
  }

  public void disabledPeriodic() {

  }

  public void disabledInit() {
    
  }

  public void testInit() {
  }
}
