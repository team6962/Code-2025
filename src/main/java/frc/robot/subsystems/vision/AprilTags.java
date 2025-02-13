package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Field;
import io.limelightvision.LimelightHelpers;
import io.limelightvision.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class AprilTags extends SubsystemBase {
  private static record BestEstimate(
      Pose2d pose, Time timestamp, Distance translationError, Angle rotationError, int tagCount) {
    public BestEstimate() {
      this(
          new Pose2d(),
          Seconds.of(Timer.getFPGATimestamp()),
          Meters.of(Double.POSITIVE_INFINITY),
          Rotations.of(Double.POSITIVE_INFINITY),
          0);
    }
  }

  public static void injectVisionData(Map<String, Pose3d> cameraPoses, SwerveDrive swerveDrive) {
    List<LimelightHelpers.PoseEstimate> poseEstimates =
        cameraPoses.keySet().stream()
            .map(LimelightHelpers::getBotPoseEstimate_wpiBlue)
            .filter((estimate) -> estimate != null)
            .collect(Collectors.toList());

    BestEstimate bestPoseEstimate = new BestEstimate();

    List<Pose2d> loggedVisionPoses = new ArrayList<>();

    for (PoseEstimate poseEstimate : poseEstimates) {
      Pose2d pose2d = poseEstimate.pose;
      if (IntStream.of(LIMELIGHT.BLACKLISTED_APRILTAGS)
          .anyMatch(x -> x == poseEstimate.rawFiducials[0].id)) continue;
      if (poseEstimate.tagCount == 0) continue;
      if (pose2d.getTranslation().getNorm() == 0.0) continue;
      if (pose2d.getRotation().getRadians() == 0.0) continue;
      if (Double.isNaN(poseEstimate.avgTagDist)) continue;

      if (pose2d.getX() < 0.0
          || pose2d.getY() < 0.0
          || pose2d.getX() > Field.LENGTH
          || pose2d.getY() > Field.WIDTH) continue;
      boolean canChangeHeading = false;
      if (poseEstimate.tagCount >= 2 || RobotState.isDisabled()) {
        canChangeHeading = true;
      }

      canChangeHeading =
          canChangeHeading
              && swerveDrive
                      .getEstimatedPose()
                      .getTranslation()
                      .getDistance(pose2d.getTranslation())
                  < 1.0;
      // if (canChangeHeading) LEDs.setState(LEDs.State.HAS_VISION_TARGET_SPEAKER);

      double rotationError = Units.degreesToRadians(15);
      if (!canChangeHeading) {
        rotationError = 9999999;
        pose2d =
            new Pose2d(
                pose2d.getTranslation(),
                swerveDrive
                    .getEstimatedPose(Seconds.of(poseEstimate.timestampSeconds))
                    .getRotation());
      }

      double translationError =
          Math.pow(Math.abs(poseEstimate.avgTagDist), 2.0)
              / Math.pow(poseEstimate.tagCount, 2)
              / 10;

      loggedVisionPoses.add(pose2d);
      translationError += 0.5;

      if (translationError < bestPoseEstimate.translationError.in(Meters)
          || rotationError < Units.degreesToRadians(360.0)) {
        bestPoseEstimate =
            new BestEstimate(
                pose2d,
                Seconds.of(poseEstimate.timestampSeconds),
                Meters.of(translationError),
                Rotations.of(rotationError),
                poseEstimate.tagCount);
      }
    }

    if ((int) bestPoseEstimate.tagCount > 0) {
      swerveDrive
          .getPoseEstimator()
          .addVisionMeasurement(
              bestPoseEstimate.pose,
              bestPoseEstimate.timestamp,
              VecBuilder.fill(
                  bestPoseEstimate.translationError.in(Meters),
                  bestPoseEstimate.translationError.in(Meters),
                  bestPoseEstimate.rotationError.in(Radians)));
    }

    Logger.getField().getObject("visionPosese").setPoses(loggedVisionPoses);
  }

  public static void printConfig(Map<String, Pose3d> cameraPoses) {
    System.out.println(
        """


//////////////////////////////////////
///// LIMELIGHT POSITION CONFIG //////
//////////////////////////////////////
""");
    for (Map.Entry<String, Pose3d> limelight : cameraPoses.entrySet()) {
      System.out.println(
          String.format(
              """
          ----- %s.local:5801 -----
            LL Forward: %.5f
            LL Right:   %.5f
            LL Up:      %.5f
            LL Roll:    %.5f
            LL Pitch:   %.5f
            LL Yaw:     %.5f
          """,
              limelight.getKey(),
              limelight.getValue().getTranslation().getX(),
              limelight.getValue().getTranslation().getY(),
              limelight.getValue().getTranslation().getZ(),
              Units.radiansToDegrees(limelight.getValue().getRotation().getX()),
              Units.radiansToDegrees(limelight.getValue().getRotation().getY()),
              Units.radiansToDegrees(limelight.getValue().getRotation().getZ())));
    }
  }

  
  public static int findClosestReefTagID() {
    int ftagID = (int) LimelightHelpers.getFiducialID(LIMELIGHT.APRILTAG_CAMERA_POSES.keySet().toArray()[0].toString());
    int btagID = (int) LimelightHelpers.getFiducialID(LIMELIGHT.APRILTAG_CAMERA_POSES.keySet().toArray()[1].toString());

    if (Field.REEF_APRILTAGS.contains(ftagID)) return ftagID;
    if (Field.REEF_APRILTAGS.contains(btagID)) return btagID;
    return -1;
  }
}
