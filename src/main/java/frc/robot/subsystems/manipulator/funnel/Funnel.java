package frc.robot.subsystems.manipulator.funnel;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ENABLED_SYSTEMS;

public abstract class Funnel extends SubsystemBase {
  public Funnel() {
    setName("Funnel");
  }

  public abstract Command intake();

  public abstract Command stop();

  public Command forwards() {
    return Commands.none();
  }

  public static Funnel create() {
    if (!ENABLED_SYSTEMS.isFunnelEnabled()) return SimFunnel.disabled();
    else if (RobotBase.isReal()) return new RealFunnel();
    else return SimFunnel.simulated();
  }
}
