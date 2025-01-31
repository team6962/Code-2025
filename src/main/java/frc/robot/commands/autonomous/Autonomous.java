package frc.robot.commands.autonomous;

import com.pathplanner.lib.path.GoalEndState;
import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;

public class Autonomous extends SequentialCommandGroup {
    private RobotStateController controller;
    private SwerveDrive swerveDrive;
    private Manipulator manipulator;
    private Intake intake;
    private Elevator elevator;

    public Autonomous(RobotStateController controller, SwerveDrive swerveDrive, Manipulator manipulator, Elevator elevator, Intake intake) {
        this.controller = controller;
        this.swerveDrive = swerveDrive;
        this.manipulator = manipulator;
        this.intake = intake;
        this.elevator = elevator;

        addCommands(pathfindToProcessor());
        addCommands(pickupPreplacedAlgae(0, AlgaePickupMechanism.INTAKE));
        addCommands(pathfindToProcessor());
        addCommands(pickupPreplacedAlgae(1, AlgaePickupMechanism.MANIPULATOR));
        addCommands(pathfindToProcessor());
        addCommands(pickupPreplacedAlgae(2, AlgaePickupMechanism.INTAKE));
        addCommands(pathfindToProcessor());
    }

    public Command pathfindToProcessor() {
        return swerveDrive.pathfindTo(new Pose2d(6.172, 0.508, Rotation2d.fromDegrees(-90)));
    }

    public Command pathfindToTopCoralStation() {
        return swerveDrive.pathfindTo(new Pose2d(1.1, 7.0, Rotation2d.fromDegrees(135)));
    }

    public Command pathfindToTopLeftReefPoles() {
        return swerveDrive.pathfindTo(new Pose2d(3.45, 5.4, Rotation2d.fromDegrees(-60)));
    }

    public Command scoreCoral() {
        return null;
    }

    public Command coralStation(){
        return null;
    }

    private static Translation2d ALGAE_SETUP = new Translation2d(1.98, 2.18);
    private static Translation2d ALGAE_DRIVE_OVER = new Translation2d(1.21, 2.18);

    public static enum AlgaePickupMechanism { INTAKE, MANIPULATOR }

    /**
     * Pickup a pre-placed algae on the ground
     * 
     * @param algaePosition The position of the algae to pickup (0 is nearest
     * the processor, and 2 is farthest)
     * @return
     */
    public Command pickupPreplacedAlgae(int algaePosition, AlgaePickupMechanism mechanism) {
        Translation2d offset = new Translation2d(0, 1.83 * algaePosition);
        Rotation2d angle = Rotation2d.fromDegrees(mechanism == AlgaePickupMechanism.MANIPULATOR ? -180 : 0);

        Command setupCommand = swerveDrive.pathfindTo(new Pose2d(ALGAE_SETUP.plus(offset), angle), new GoalEndState(5, angle));
        Command driveOverCommand = swerveDrive.pathfindTo(new Pose2d(ALGAE_DRIVE_OVER.plus(offset), angle));

        if (mechanism == AlgaePickupMechanism.INTAKE) {
            return Commands.sequence(
                setupCommand,
                Commands.deadline(
                    driveOverCommand,
                    intake.wheels.intake(),
                    intake.pivot.lower()
                ),
                intake.pivot.raise()
            );
        } else {
            return Commands.sequence(
                setupCommand,
                driveOverCommand
            );
        }
    }
    
    public boolean hasCoral() {
        return false;
    }
}
