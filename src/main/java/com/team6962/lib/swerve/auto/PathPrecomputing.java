package com.team6962.lib.swerve.auto;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team6962.lib.prepath.CustomLocalADStar;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.MeasureMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CachedRobotState;
import java.util.LinkedList;
import java.util.List;

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

    public Pose2d getStartPose() {
      return startPose;
    }

    public Pose2d getEndPose() {
      return endPose;
    }

    public boolean isAvailable() {
      return path != null && trajectory != null;
    }

    public boolean isStartPoseNear(Pose2d otherPose) {
      return startPose.getTranslation().getDistance(otherPose.getTranslation())
              < Units.inchesToMeters(12)
          && MeasureMath.minAbsDifference(startPose.getRotation(), otherPose.getRotation())
                  .getDegrees()
              < 30;
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
      PathPlannerPath path =
          Pathfinding.getCurrentPath(
              constraints, new GoalEndState(0, current.endPose.getRotation()));

      if (path == null) return;

      PathPoint lastPoint = null;

      for (PathPoint point : path.getAllPathPoints()) {
        if (point != null) lastPoint = point;
      }

      if (lastPoint == null) return;

      if (lastPoint.position.getDistance(current.endPose.getTranslation())
          > Units.inchesToMeters(3)) {
        System.out.println(
            "Last path point is too far from the end pose, so the path will not be used");
        return;
      }

      Waypoint lastWaypoint = path.getWaypoints().get(path.getWaypoints().size() - 1);

      if (lastWaypoint == null) return;
      if (lastWaypoint.anchor().getDistance(current.endPose.getTranslation())
          > Units.inchesToMeters(3)) {
        System.out.println(
            "Last waypoint is too far from the end pose, so the path will not be used");
        return;
      }

      current.path = path;

      if (current.path != null) {
        current.trajectory =
            new PathPlannerTrajectory(
                current.path, new ChassisSpeeds(), current.startPose.getRotation(), robot);
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

  private void log(String key, Precompute precompute) {
    if (precompute == null) {
      Logger.log(key, "null");
      return;
    }

    Logger.log(key, "");

    Logger.log(key + "/startPose", precompute.startPose);
    Logger.log(key + "/endPose", precompute.endPose);

    if (precompute.path != null) {
      List<Waypoint> waypoints = precompute.path.getWaypoints();

      for (int i = 0; i < waypoints.size(); i++) {
        Logger.log(key + "/waypoint" + i, waypoints.get(i).anchor());
      }
    }
  }

  private void log(String key, List<Precompute> precomputes) {
    for (int i = 0; i < precomputes.size(); i++) {
      log(key + "/precompute" + i, precomputes.get(i));
    }

    Logger.log(key + "/size", precomputes.size());
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

    log("PathPrecomputing/current", current);
    log("PathPrecomputing/queue", queue);

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
