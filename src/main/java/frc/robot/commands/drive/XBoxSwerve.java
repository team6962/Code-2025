// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Preferences;
import frc.robot.subsystems.RobotStateController;
import frc.robot.util.software.MathUtils;
import frc.robot.util.software.MathUtils.InputMath;

public class XBoxSwerve extends Command {
  private XboxController controller;
  private SwerveDrive swerveDrive;
  private RobotStateController stateController;

  public double MAX_DRIVE_VELOCITY;
  public double NOMINAL_DRIVE_VELOCITY;
  public double FINE_TUNE_DRIVE_VELOCITY;
  public double NOMINAL_ANGULAR_VELOCITY;
  public double MAX_ANGULAR_VELOCITY;
  
  private Translation2d velocity = new Translation2d();
  private double angularVelocity = 0.0;

  private Command driveCommand;
  
  public XBoxSwerve(SwerveDrive swerveDrive, XboxController xboxController, RobotStateController stateController) {
    this.swerveDrive = swerveDrive;
    this.controller = xboxController;
    this.stateController = stateController;
    // controller.setRumble(RumbleType.kBothRumble, 1.0);
    addRequirements(swerveDrive);

    MAX_DRIVE_VELOCITY = swerveDrive.getLinearDriveVelocity(Preferences.SWERVE_DRIVE.TELEOPERATED_BOOST_POWER).in(MetersPerSecond);
    NOMINAL_DRIVE_VELOCITY = swerveDrive.getLinearDriveVelocity(Preferences.SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER).in(MetersPerSecond);
    FINE_TUNE_DRIVE_VELOCITY = swerveDrive.getLinearDriveVelocity(Preferences.SWERVE_DRIVE.TELEOPERATED_FINE_TUNE_DRIVE_POWER).in(MetersPerSecond);
    NOMINAL_ANGULAR_VELOCITY = swerveDrive.getAngularDriveVelocity(Preferences.SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER).in(RadiansPerSecond);
    MAX_ANGULAR_VELOCITY = swerveDrive.getAngularDriveVelocity(Preferences.SWERVE_DRIVE.TELEOPERATED_BOOST_POWER).in(RadiansPerSecond);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotState.isTeleop()) return;

    // Disable drive if the controller disconnects
    if (!controller.isConnected()) {
      if (driveCommand != null) driveCommand.cancel();

      return;
    }

    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    Translation2d leftStick = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    Translation2d rightStick = new Translation2d(controller.getRightX(), -controller.getRightY());
    
    if (RobotBase.isSimulation()) {
      leftStick = new Translation2d(controller.getRawAxis(0), -controller.getRawAxis(1));
      rightStick = new Translation2d(controller.getRawAxis(2), -controller.getRawAxis(3));
      leftTrigger = controller.getRawAxis(5);
      rightTrigger = controller.getRawAxis(4);
    }

    // Deadbands
    leftStick = InputMath.addCircularDeadband(leftStick, 0.1);
    rightStick = InputMath.addCircularDeadband(rightStick, 0.1);


    angularVelocity += -rightStick.getX() * MathUtils.map(rightTrigger, 0, 1, NOMINAL_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    
    velocity = velocity.plus(leftStick.times(MathUtils.map(rightTrigger, 0, 1, NOMINAL_DRIVE_VELOCITY, MAX_DRIVE_VELOCITY)));

    if (controller.getPOV() != -1) {
      Translation2d povVelocity = new Translation2d(Math.cos(Units.degreesToRadians(controller.getPOV())) * FINE_TUNE_DRIVE_VELOCITY, -Math.sin(Units.degreesToRadians(controller.getPOV())) * FINE_TUNE_DRIVE_VELOCITY);
      velocity = velocity.plus(povVelocity);
    }

    // Zero heading when Y is pressed
    if (controller.getYButton()) {
      Rotation2d newHeading = new Rotation2d();
      if (!Constants.IS_BLUE_TEAM.get()) {
        newHeading = Rotation2d.fromDegrees(180.0);
      } else if (Constants.IS_BLUE_TEAM.get()) {
        newHeading = Rotation2d.fromDegrees(0);
      }
      swerveDrive.getGyroscope().setHeading(newHeading.getMeasure());
    }

    if (controller.getAButton()) {
      // swerveDrive.goToNearestPose(List.of(Field.AUTO_MOVE_POSITIONS.values().toArray(new Pose2d[] {})), controller).schedule();
    }

    if (RobotBase.isSimulation()) {
      if (Constants.IS_BLUE_TEAM.get()) {
        velocity = velocity.rotateBy(Rotation2d.fromDegrees(90.0));
      } else {
        velocity = velocity.rotateBy(Rotation2d.fromDegrees(-90.0));
      }
    }

    ChassisSpeeds drivenSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), angularVelocity);

    Logger.log("XBoxSwerve/drivenSpeeds", drivenSpeeds);

    driveCommand = swerveDrive.drive(drivenSpeeds);
    driveCommand.schedule();

    // if (leftStick.getNorm() > 0.05 && (controller.getLeftBumper() || controller.getRightBumper())) {
    //   swerveDrive.setTargetHeading(leftStick.getAngle());
    // }
    
    angularVelocity = 0.0;
    velocity = new Translation2d();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) driveCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}