package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.Constants.ALGAE;
import frc.robot.Constants.Constants.PHOTONVISION;
import frc.robot.subsystems.LEDs;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class photonAlgae {
  private static final double MAX_FOV_RATIO = Math.PI / 2;

  public static Translation2d getAlgaePosition(
      String name, SwerveDrive swerveDrive, Translation3d cameraToRobot) {

    try (PhotonCamera camera = new PhotonCamera(name)) {
      PhotonPipelineResult result = camera.getLatestResult();

      if (result == null || !result.hasTargets()) return null; // Check if detection is valid

      PhotonTrackedTarget target = result.getBestTarget();
      if (target == null || target.getDetectedObjectClassID() != 1)
        return null; // Check if target is valid

      Translation2d algaePosition = new Translation2d(0, 0);

      Angle horizontalOffset = Degrees.of(target.getYaw());

      double pixelHeight = photonAlgae.getTargetHeight(target);
      double pixelWidth = photonAlgae.getTargetWidth(target);
      double pixelRatio = pixelHeight / pixelWidth;

      if (!isValidSphericalObject(pixelHeight, pixelWidth, pixelRatio)) return null;

      double targetFOVRatio =
          PHOTONVISION.FOV_HEIGHT.getRadians()
              * (pixelHeight / PHOTONVISION.ALGAE_CAMERA_HEIGHT_PIXELS);
      if (targetFOVRatio > MAX_FOV_RATIO) return null;

      Distance distance = calculateDistance(targetFOVRatio);
      if (distance.gt(PHOTONVISION.MAX_DETECTION_RANGE)) return null;

      Translation2d relativePosition =
          calculateRelativePosition(distance, horizontalOffset, cameraToRobot);

      Time timestamp = Microseconds.of(result.getTimestampSeconds());
      Pose2d robotPosition = swerveDrive.getEstimatedPose(timestamp);
      algaePosition =
          robotPosition
              .getTranslation()
              .plus(relativePosition.rotateBy(robotPosition.getRotation()));

      if (!RobotState.isDisabled()) {
        LEDs.setState(LEDs.State.CAN_SEE_ALGAE);
      }
      return algaePosition;
    }
  }

  // 0.0 is the NaN/Null value here
  private static double getTargetHeight(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0.0; // Ensure there are enough corners

    corners.sort((a, b) -> Double.compare(a.y, b.y));
    double avgMinY = (corners.get(0).y + corners.get(1).y) / 2.0;
    double avgMaxY = (corners.get(corners.size() - 1).y + corners.get(corners.size() - 2).y) / 2.0;

    double height = avgMaxY - avgMinY;
    return Double.isNaN(height) ? 0.0 : height; // Ensure NaN is handled
  }

  // 0.0 is the NaN/Null value here
  private static double getTargetWidth(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0.0; // Ensure there are enough corners

    corners.sort((a, b) -> Double.compare(a.x, b.x));
    double avgMinX = (corners.get(0).x + corners.get(1).x) / 2.0;
    double avgMaxX = (corners.get(corners.size() - 1).x + corners.get(corners.size() - 2).x) / 2.0;

    double width = avgMaxX - avgMinX;
    return Double.isNaN(width) ? 0.0 : width; // Ensure NaN is handled
  }

  private static boolean isValidSphericalObject(
      double pixelHeight, double pixelWidth, double pixelRatio) {
    if (pixelHeight <= 0 || pixelWidth <= 0) return false;
    return Math.abs(pixelRatio - 1.0) <= PHOTONVISION.SPHERE_TOLERANCE;
  }

  private static Distance calculateDistance(double targetFOVRatio) {
    Distance absoluteDistance =
        Meters.of((ALGAE.ALGAE_DIAMETER.in(Meters) / 2) / Math.tan(targetFOVRatio));
    return Meters.of(
        Math.sqrt(
            Math.pow(absoluteDistance.in(Meters), 2)
                - Math.pow(
                    Meters.of(PHOTONVISION.ALGAE_CAMERA_POSITION.getY()).in(Meters)
                        - (ALGAE.ALGAE_DIAMETER.in(Meters)) / 2,
                    2)));
  }

  private static Translation2d calculateRelativePosition(
      Distance distance, Angle horizontalOffset, Translation3d cameraToRobot) {
    return new Translation2d(
            distance.in(Meters) * Math.cos(horizontalOffset.in(Radians)),
            -distance.in(Meters) * Math.sin(horizontalOffset.in(Radians)))
        .plus(cameraToRobot.toTranslation2d());
  }
}
