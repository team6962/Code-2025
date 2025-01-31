package frc.robot.subsystems;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

// This class is a subsystem that controls the state of the robot. It is used to coordinate the actions of the intake, shooter, transfer, and amp subsystems.

public class RobotStateController extends SubsystemBase {
  private SwerveDrive swerveDrive;
  // private static ShuffleboardTab tab = Shuffleboard.getTab("Auto");
  // private static SimpleWidget hasNote = tab.add("has Note", true).withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(0, 0);

  public RobotStateController(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;

    Logger.logNumber(getName() + "/Loop Time", () -> Robot.getLoopTime());
    Logger.logNumber(getName() + "/Compute Time", () -> Robot.getComputeTime());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
