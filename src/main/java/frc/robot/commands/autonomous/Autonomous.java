package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.RobotStateController;

public class Autonomous extends Command {
    private RobotStateController controller;
    private SwerveDrive swerveDrive;
    
    // private final ExampleSubsystem m_subsystem;

    public Autonomous(ExampleSubsystem subsystem, RobotStateController controller, SwerveDrive swerveDrive) {
        this.controller = controller;
        this.swerveDrive = swerveDrive;

        // for (int i = 0; i < 6; i ++) {
        //     Pose2d aprilTagPose = new Pose2d(
        //         (176.745 + Math.cos(i * Math.PI / 3) * 32.75), // x
        //         (158.5 + Math.sin(i * Math.PI / 3) * 32.75), // y
        //         new Rotation2d(Math.PI + (Math.PI / 3) * i) // angle
        //     ); 
        // }


        // addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
