package frc.robot.commands.autonomous;

import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotStateController;

public class Autonomous {
    private RobotStateController controller;
    private SwerveDrive swerveDrive;

    public Autonomous(RobotStateController controller, SwerveDrive swerveDrive) {
        this.controller = controller;
        this.swerveDrive = swerveDrive;
    }

    public Command processor(){
        return null;
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
