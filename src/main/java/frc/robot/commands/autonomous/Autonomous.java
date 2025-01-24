package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.RobotStateController;

public class Autonomous extends Command {
    private RobotStateController controller;
    private SwerveDrive swerveDrive;
    private Command runningCommand;
    private List<Integer> reefFaces = List.of();
    private boolean simulatedCoral = true;
    private boolean startingAlgae = false;
    private boolean rightCoralStation = false;
    private boolean leftCoralStation = false;

    private enum State {
        PLACE_CORAL,
        PICKUP_ALGAE,
        CORAL_STATION,
        PROCESSOR,
      }
    
    public State state;
    
    // private final ExampleSubsystem m_subsystem;

    public Debouncer hasCoralDebouncer = new Debouncer(0.1, DebounceType.kFalling);


    public Autonomous(RobotStateController controller, SwerveDrive swerveDrive, List<Integer> reefFaces, boolean startAlgae, boolean leftStation, boolean rightStaiton) {
        this.controller = controller;
        this.swerveDrive = swerveDrive;
        this.startingAlgae = startAlgae;
        this.leftCoralStation = leftStation;
        this.rightCoralStation = rightStaiton;
        this.reefFaces = reefFaces;
    }

    @Override
    public void initialize() {
        this.state = null;
        this.simulatedCoral = true;
    }

    @Override
    public void execute() {
        if (!RobotState.isAutonomous()) {
            runningCommand.cancel();
            this.cancel();
            return;
          }
        
        if (startingAlgae && (state != State.PROCESSOR || !runningCommand.isScheduled())) {
            if (runningCommand != null) runningCommand.cancel();
            runningCommand = processor();
            runningCommand.schedule();
            state = State.PROCESSOR;
            System.out.println("SCORING PROCESSOR");
            return;
        }

        if ((state != State.PLACE_CORAL || !runningCommand.isScheduled()) && hasCoral() && !reefFaces.isEmpty()) {
            if (runningCommand != null) runningCommand.cancel();
            runningCommand = scoreCoral();
            runningCommand.schedule();
            state = State.PLACE_CORAL;
            System.out.println("PLACING CORAL");
            return;
        }

        if ((state != State.CORAL_STATION || !runningCommand.isScheduled()) && !hasCoral() && (leftCoralStation || rightCoralStation)) {
            state = State.CORAL_STATION;
            if (runningCommand != null) runningCommand.cancel();
            runningCommand = coralStation();
            runningCommand.schedule();
            System.out.println("PICKING UP FROM CORAL STATION");
            return;
        }


        if (state != State.PICKUP_ALGAE || !runningCommand.isScheduled()) {
            state = State.PICKUP_ALGAE;
            if (runningCommand != null) runningCommand.cancel();
            runningCommand = pickupAlgae();
            runningCommand.schedule();
            System.out.println("PICKING UP ALGAE");
            return;
        }

        
        
    }

    public Command processor(){
        return null;
    }

    public Command scoreCoral() {
        return null;
        // for (int i = 0; i < 6; i ++) {
        //     Pose2d aprilTagPose = new Pose2d(
        //         (176.745 + Math.cos(i * Math.PI / 3) * 32.75), // x
        //         (158.5 + Math.sin(i * Math.PI / 3) * 32.75), // y
        //         new Rotation2d(Math.PI + (Math.PI / 3) * i) // angle
        //     ); 
        // }


        // addRequirements(subsystem);
    }

    public Command coralStation(){
        return null;
    }


    public Command pickupAlgae() {
        return null;
        //we have no beam breaks in the algae manipulator.
    }
    
    public boolean hasCoral() {
        // if (RobotBase.isSimulation()) {
        //     return hasCoralDebouncer.calculate(simulatedCoral);
        // // return hasNote.getEntry().getBoolean(false);
        // } else {
        //     return hasCoralDebouncer.calculate(controller.hasCoral());
        // }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
