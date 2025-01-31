package frc.robot.commands.autonomous;

import static edu.wpi.first.units.Units.Degrees;

import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotStateController;

public class Autonomous extends SequentialCommandGroup {
    private RobotStateController controller;
    private SwerveDrive swerveDrive;

    public Autonomous(RobotStateController controller, SwerveDrive swerveDrive) {
        this.controller = controller;
        this.swerveDrive = swerveDrive;

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


    public Command pickupAlgae() {
        return null;
    }
    
    public boolean hasCoral() {
        return false;
    }
}
