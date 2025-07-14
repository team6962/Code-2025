package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.Constants.ELEVATOR;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.util.hardware.motion.BasedElevator;
import frc.robot.util.hardware.motion.SimElevator;

public class TrueElevator {
  public static Elevator create() {
    if (RobotBase.isSimulation() || !ENABLED_SYSTEMS.isElevatorEnabled()) {
      return new Sim();
    } else {
      return new Real();
    }
  }

  private static class Real extends BasedElevator implements Elevator {
    public Real() {
      super("Elevator", ELEVATOR.CONFIG);
    }
  }

  private static class Sim extends SimElevator implements Elevator {
    public Sim() {
      super("Elevator", ELEVATOR.CONFIG);
    }
  }
}
