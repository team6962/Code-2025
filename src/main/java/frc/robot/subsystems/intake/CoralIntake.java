package frc.robot.subsystems.intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.RobotVersion;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.subsystems.manipulator.coral.CoralGrabber;

/**
 * An interface representing a subsystem that can intake coral. For the version 1
 * robot, this controls the manipulator with {@link GrabberIntake}. For
 * the version 2 robot, this controls the funnel with {@link FunnelIntake}.
 */
public interface CoralIntake {
    /**
     * Creates a command to intake a coral, finishing when the coral is detected
     * to be in the robot, so the robot can drive away from the coral station.
     * @return The command.
     */
    public Command intake();

    /**
     * Returns if it is unsafe to move the manipulator because a coral is being
     * transferred between the funnel and the manipulator.
     * @return True if unsafe, false otherwise.
     */
    public boolean isUnsafeToMoveManipulator();

    /**
     * Creates a command to fine control the coral intake at a speed from -1
     * (reverse) to 1 (intaking).
     * @param power The power to run the intake at.
     * @return The command.
     */
    public Command fineControl(double power);

    public Set<Subsystem> getSubsystems();

    /**
     * Creates a new {@link CoralIntake} based on the robot version.
     * @param coralGrabber The {@link CoralGrabber} to use for the intake, if the
     * robot is version 1.
     * @return A {@link GrabberIntake}, if the robot is version 1, or a
     * {@link FunnelIntake}, if the robot is version 2.
     */
    public static CoralIntake get(CoralGrabber coralGrabber) {
        if (RobotVersion.isV2()) return ENABLED_SYSTEMS.FUNNEL ? new FunnelIntake() : new DisabledIntake();
        else return ENABLED_SYSTEMS.MANIPULATOR ? new GrabberIntake(coralGrabber) : new DisabledIntake();
    }
}
