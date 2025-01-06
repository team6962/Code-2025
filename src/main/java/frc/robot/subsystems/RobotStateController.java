package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.Preferences;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

// This class is a subsystem that controls the state of the robot. It is used to coordinate the actions of the intake, shooter, transfer, and amp subsystems.

public class RobotStateController extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private DigitalInput beamBreakSensor;
  private Debouncer beamBreakDebouncer = new Debouncer(0.1);
  private Debouncer shotDebouncer = new Debouncer(0.25);
  private boolean shootOverride = false;
  // private static ShuffleboardTab tab = Shuffleboard.getTab("Auto");
  // private static SimpleWidget hasNote = tab.add("has Note", true).withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(0, 0);



  public enum State {
  }

  public RobotStateController(SwerveDrive swerveDrive) {

    StatusChecks.addCheck(new SubsystemBase() {}, "Beam Break Sensor", () -> beamBreakSensor.get());

    Logger.autoLog(this, "Loop Time", () -> Robot.getLoopTime());
    Logger.autoLog(this, "Compute Time", () -> Robot.getComputeTime());
  }

  /**
   * Sets the state of the robot
   * @param state
   * @return
   */

  public Command setState(State state) {
    return new Command() {};
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
