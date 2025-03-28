package com.team6962.lib.swerve.auto;

import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team6962.lib.prepath.CustomLocalADStar;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CachedRobotState;

public class PathPrecomputing extends SubsystemBase {
    private Precompute current;
    private List<Precompute> queue = new LinkedList<>();
    private PathConstraints constraints;
    private RobotConfig robot;
    private CustomLocalADStar pathfinder = CustomLocalADStar.getInstance();

    public PathPrecomputing(PathConstraints constraints, RobotConfig robot) {
        this.constraints = constraints;
        this.robot = robot;

        CustomLocalADStar.usePathfinder();
    }
    
    public static class Precompute {
        PathPlannerPath path;
        PathPlannerTrajectory trajectory;
        Pose2d startPose;
        Pose2d endPose;

        public PathPlannerPath getPath() {
            return path;
        }

        public PathPlannerTrajectory getTrajectory() {
            return trajectory;
        }

        public boolean isAvailable() {
            return path != null && trajectory != null;
        }

        public boolean isStartPoseNear(Pose2d otherPose) {
            return startPose.getTranslation().getDistance(otherPose.getTranslation()) < Units.inchesToMeters(6) &&
                   MeasureMath.minAbsDifference(startPose.getRotation(), otherPose.getRotation()).getDegrees() < 10;
        }
    }

    private void next() {
        if (!queue.isEmpty()) {
            current = queue.remove(0);
            
            Pathfinding.ensureInitialized();
            Pathfinding.setStartPosition(current.startPose.getTranslation());
            Pathfinding.setGoalPosition(current.endPose.getTranslation());
        } else {
            current = null;
        }
    }

    private void update() {
        if (Pathfinding.isNewPathAvailable()) {
            current.path = Pathfinding.getCurrentPath(constraints, new GoalEndState(0, current.endPose.getRotation()));

            if (current.path != null) {
                current.trajectory = new PathPlannerTrajectory(current.path, new ChassisSpeeds(), current.startPose.getRotation(), robot);
            }
        }
    }

    public Precompute precompute(Pose2d startPose, Pose2d endPose) {
        Precompute precompute = new Precompute();
        precompute.startPose = startPose;
        precompute.endPose = endPose;

        if (pathfinder.willFindPath(startPose.getTranslation(), endPose.getTranslation())) {
            queue.add(precompute);
        } else {
            System.out.println("Start and end poses are too close, so a path will not be generated");
        }

        return precompute;
    }

    @Override
    public void periodic() {
        if (CachedRobotState.isEnabled()) {
            if (current != null) {
                queue.add(0, current);
                current = null;
            }

            return;
        }
        
        if (current == null || current.trajectory != null) {
            if (current != null) {
                System.out.println("Done with current path, moving to next");
                System.out.println(current.startPose);
                System.out.println(current.endPose);
                System.out.println(queue.size());
            }

            next();
        }

        if (current != null) {
            update();
        }
    }
}
