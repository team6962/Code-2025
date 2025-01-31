package frc.robot.commands.autonomous;

import static edu.wpi.first.units.Units.Degrees;

import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotStateController;

public class Autonomous extends SequentialCommandGroup {
    private RobotStateController controller;
    private SwerveDrive swerveDrive;

    public Autonomous(RobotStateController controller, SwerveDrive swerveDrive) {
        this.controller = controller;
        this.swerveDrive = swerveDrive;

        addCommands(pathfindToProcessor());
        addCommands(pickupPreplacedAlgae(0));
        addCommands(pathfindToProcessor());
        addCommands(pickupPreplacedAlgae(1));
        addCommands(pathfindToProcessor());
        addCommands(pickupPreplacedAlgae(2));
        addCommands(pathfindToProcessor());
    }

    public Command pathfindToProcessor() {
        return swerveDrive.pathfindTo(new Pose2d(6.172, 0.508, Rotation2d.fromDegrees(-90)));
    }

    public Command scoreCoral() {
        return null;
    }

    public Command coralStation(){
        return null;
    }

    private static Translation2d ALGAE_PICKUP_1 = new Translation2d(1.98, 2.18);
    private static Translation2d ALGAE_PICKUP_2 = new Translation2d(1.21, 2.18);

    /**
     * Pickup a pre-placed algae on the ground
     * 
     * @param algaePosition The position of the algae to pickup (0 is nearest
     * the processor, and 2 is farthest)
     * @return
     */
    public Command pickupPreplacedAlgae(int algaePosition) {
        Translation2d offset = new Translation2d(0, 1.83 * algaePosition);

        return Commands.sequence(
            swerveDrive.pathfindTo(new Pose2d(ALGAE_PICKUP_1.plus(offset), Rotation2d.fromDegrees(-180))),
            swerveDrive.pathfindTo(new Pose2d(ALGAE_PICKUP_2.plus(offset), Rotation2d.fromDegrees(-180)))
        );
    }
    
    public boolean hasCoral() {
        return false;
    }
}
