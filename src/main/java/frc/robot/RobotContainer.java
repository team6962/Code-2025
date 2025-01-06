// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.commands.drive.WheelRadiusCalibration;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.AprilTags;
import frc.robot.util.software.Dashboard.AutonChooser;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  // The robot's subsystems and commands
  private final SwerveDrive swerveDrive;
  private final RobotStateController stateController;
  private final LEDs ledStrip;
  // private final CollisionDetector collisionDetector;

  private static PowerDistribution PDH = new PowerDistribution(CAN.PDH, ModuleType.kRev);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    // Logger.autoLog("PDH", PDH);

    Logger.startLog();
    AutonChooser.init();
    
    LiveWindow.disableAllTelemetry();
    
    DriverStation.silenceJoystickConnectionWarning(true);

    StatusChecks.addCheck(new SubsystemBase() {}, "FMS Attached", () -> DriverStation.isFMSAttached());
    StatusChecks.addCheck(new SubsystemBase() {}, "DS Attached", () -> DriverStation.isDSAttached());
    StatusChecks.addCheck(new SubsystemBase() {}, "Joystick 0", () -> DriverStation.isJoystickConnected(0));
    StatusChecks.addCheck(new SubsystemBase() {}, "Joystick 1", () -> DriverStation.isJoystickConnected(1));
    StatusChecks.addCheck(new SubsystemBase() {}, "Battery Voltage", () -> RobotController.getBatteryVoltage() > 12.0);
    // StatusChecks.addCheck(new SubsystemBase() {}, "RSL", () -> RobotController.getRSLState());
    StatusChecks.addCheck(new SubsystemBase() {}, "Loop Time", () -> Robot.getComputeTime() <= 0.02);
    StatusChecks.addCheck(new SubsystemBase() {}, "3V3 Enabled", () -> RobotController.getEnabled3V3());
    StatusChecks.addCheck(new SubsystemBase() {}, "5V Enabled", () -> RobotController.getEnabled5V());
    StatusChecks.addCheck(new SubsystemBase() {}, "6V Enabled", () -> RobotController.getEnabled6V());
    StatusChecks.addCheck(new SubsystemBase() {}, "Sys Time Valid", () -> RobotController.isSystemTimeValid());

    swerveDrive = new SwerveDrive();
    stateController = new RobotStateController(swerveDrive);
    ledStrip = new LEDs(stateController);
    // collisionDetector = new CollisionDetector();
    
    // Configure the trigger bindings
    Controls.configureBindings(stateController, swerveDrive);

    SwerveDrive.printChoreoConfig();
    AprilTags.printConfig(Constants.LIMELIGHT.APRILTAG_CAMERA_POSES);

    Pathfinding.ensureInitialized();
  }

  public Command getAutonomousCommand() {
    // return new Autonomous(stateController, swerveDrive, AutonChooser.getNotes());
    return Commands.run(() -> {});
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

  public void disabledPeriodic() {

  }

  public void disabledInit() {
    
  }

  public void testInit() {
    (new WheelRadiusCalibration(swerveDrive)).schedule();
  }
}
