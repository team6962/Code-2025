package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.Constants.ALGAE;
import frc.robot.Constants.Constants.PHOTONVISION;
import frc.robot.subsystems.LEDs;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class photonAlgae {
  private static final double MAX_FOV_RATIO = Math.PI / 2;

  public static Translation2d getAlgaePosition(
      String name, SwerveDrive swerveDrive, Translation3d cameraToRobot) {
    
    PhotonCamera camera = new PhotonCamera(name);
    PhotonPipelineResult result = camera.getLatestResult();

    if (!result.hasTargets()) return null; // Check if detection is valid

    PhotonTrackedTarget target = result.getBestTarget();

    if (!(target.getDetectedObjectClassID()==1)) return null; //need to change

    Translation2d algaePosition = new Translation2d(0, 0);

    Angle horizontalOffset = Degrees.of(target.getYaw());
    
    
    double pixelHeight = target.getBoundingBox().getHeight();
    double pixelWidth = target.getBoundingBox().getWidth();
    double pixelRatio = pixelHeight / pixelWidth;

    if (!isValidSphericalObject(pixelHeight, pixelWidth, pixelRatio)) return null;

    double targetFOVRatio =
        PHOTONVISION.FOV_HEIGHT.getRadians() * (pixelHeight / PHOTONVISION.ALGAE_CAMERA_HEIGHT_PIXELS);
    if (targetFOVRatio > MAX_FOV_RATIO) return null;

    Distance distance = calculateDistance(targetFOVRatio);
    if (distance.gt(PHOTONVISION.MAX_DETECTION_RANGE)) return null;

    Translation2d relativePosition =
        calculateRelativePosition(distance, horizontalOffset, cameraToRobot);
    
    
    Time timestamp = Microseconds.of(result.getTimestampSeconds());
    Pose2d robotPosition = swerveDrive.getEstimatedPose(timestamp);
    algaePosition =
        robotPosition.getTranslation().plus(relativePosition.rotateBy(robotPosition.getRotation()));

    if (!RobotState.isDisabled()) {
      LEDs.setState(LEDs.State.CAN_SEE_ALGAE);
    }

    return algaePosition;
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
