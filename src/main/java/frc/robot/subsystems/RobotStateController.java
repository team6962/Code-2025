package frc.robot.subsystems;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveDrive;

// This class is a subsystem that controls the state of the robot. It is used to coordinate the actions of the intake, shooter, transfer, and amp subsystems.

public class RobotStateController extends SubsystemBase {
  private SwerveDrive swerveDrive;
  // private static ShuffleboardTab tab = Shuffleboard.getTab("Auto");
  // private static SimpleWidget hasNote = tab.add("has Note", true).withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(0, 0);



  public enum State {
  }

  public RobotStateController(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;

    Logger.logNumber(getName() + "/Loop Time", () -> Robot.getLoopTime());
    Logger.logNumber(getName() + "/Compute Time", () -> Robot.getComputeTime());
  }

  /**
   * Sets the state of the robot
   * @param state
   * @return
   */
  public Command setState(State state) {
    return new Command() {};
  }

  public Translation2d getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
