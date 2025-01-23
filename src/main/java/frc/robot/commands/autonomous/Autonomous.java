package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.RobotStateController;

public class Autonomous extends Command {
    private RobotStateController controller;
    private SwerveDrive swerveDrive;
    private Command runningCommand;

    private enum State {
        PLACE_CORAL,
        PICKUP_ALGAE,
        CORAL_STATION,
        PROCESSOR,
      }
    
      public State state;
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
        if (!RobotState.isAutonomous()) {
            runningCommand.cancel();
            this.cancel();
            return;
          }
        
        if ((state != State.PROCESSOR || !runningCommand.isScheduled()) && hasAlgae()) {
            if (runningCommand != null) runningCommand.cancel();
            runningCommand = processor();
            runningCommand.schedule();
            state = State.PROCESSOR;
            System.out.println("SCORING PROCESSOR");
            return;
        }

        if ((state != State.PLACE_CORAL || !runningCommand.isScheduled()) && hasCoral()) {
            if (runningCommand != null) runningCommand.cancel();
            runningCommand = scoreCoral();
            runningCommand.schedule();
            state = State.PLACE_CORAL;
            System.out.println("PLACING CORAL");
            return;
        }

        if ((state != State.CORAL_STATION || !runningCommand.isScheduled()) && !hasCoral() && (leftCoralStation() || rightCoralStation())) {
            state = State.CORAL_STATION;
            if (runningCommand != null) runningCommand.cancel();
            runningCommand = coralStation();
            runningCommand.schedule();
            System.out.println("PICKING UP FROM CORAL STATION");
            return;
        }


        if ((state != State.PICKUP_ALGAE || !runningCommand.isScheduled()) && !hasAlgae()) {
            state = State.PICKUP_ALGAE;
            if (runningCommand != null) runningCommand.cancel();
            runningCommand = pickupAlgae();
            runningCommand.schedule();
            System.out.println("PICKING UP ALGAE");
            return;
        }

        
        
    }

    public Command processor(){

    }

    public Command scoreCoral(){

    }

    public Command coralStation(){

    }

    public Command pickupAlgae(){

    }
    
    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
