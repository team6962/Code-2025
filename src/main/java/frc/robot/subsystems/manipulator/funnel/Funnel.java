package frc.robot.subsystems.manipulator.funnel;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.subsystems.manipulator.coral.CoralGrabber;

public abstract class Funnel extends SubsystemBase{
    public Funnel() {
        setName("Funnel");
    }

    public abstract Command intake(CoralGrabber coral);

    public abstract Command stop();

    public Command forwards() {
        return Commands.none();
    }

    public static Funnel create() {
        if (!ENABLED_SYSTEMS.MANIPULATOR) return SimFunnel.disabled();
        else if (RobotBase.isReal()) return new RealFunnel();
        else return SimFunnel.simulated();
    }
}