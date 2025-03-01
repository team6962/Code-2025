package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.Constants.ALGAE;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.subsystems.LEDs;
import frc.robot.util.CachedRobotState;

public class Algae {
  private static final double MAX_FOV_RATIO = Math.PI / 2;

  public static Translation2d getAlgaePosition(
      String name, SwerveDrive swerveDrive, Translation3d cameraToRobot) {
    if (RobotBase.isSimulation() && ALGAE.SIMULATE_STATIC_ALGAE) {
        return new Translation2d(2.281, 4.03);
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);

    if (table.getEntry("tv").getDouble(0) == 0) return null; // check if detection is valid

    if (!table.getEntry("tdclass").getString("").equals("algae")) return null;

    Translation2d algaePosition = new Translation2d(0, 0);

    Angle horizontalOffset =
        Degrees.of(table.getEntry("tx").getDouble(0))
            .unaryMinus(); // make all angles negative since camera feed is upside down
    double[] t2d = table.getEntry("t2d").getDoubleArray(new double[17]);
    double pixelHeight = t2d[15];
    double pixelWidth = t2d[14];
    double pixelRatio = pixelHeight / pixelWidth;

    if (!isValidSphericalObject(pixelHeight, pixelWidth, pixelRatio)) return null;

    double targetFOVRatio =
        LIMELIGHT.FOV_HEIGHT.getRadians() * (pixelHeight / LIMELIGHT.ALGAE_CAMERA_HEIGHT_PIXELS);
    if (targetFOVRatio > MAX_FOV_RATIO) return null;

    Distance distance = calculateDistance(targetFOVRatio);
    if (distance.gt(LIMELIGHT.MAX_DETECTION_RANGE)) return null;

    Translation2d relativePosition =
        calculateRelativePosition(distance, horizontalOffset, cameraToRobot);

    double latency = table.getEntry("tl").getDouble(0) + table.getEntry("cl").getDouble(0);
    double timestamp = (table.getEntry("hb").getLastChange() / 1000000.0) - (latency / 1000.0);

    Pose2d robotPosition = swerveDrive.getEstimatedPose(Seconds.of(timestamp));
    algaePosition =
        robotPosition.getTranslation().plus(relativePosition.rotateBy(robotPosition.getRotation()));

    if (!CachedRobotState.isDisabled()) {
      LEDs.setState(LEDs.State.CAN_SEE_ALGAE);
    }

    return algaePosition;
  }

  private static boolean isValidSphericalObject(
      double pixelHeight, double pixelWidth, double pixelRatio) {
    if (pixelHeight <= 0 || pixelWidth <= 0) return false;
    return Math.abs(pixelRatio - 1.0) <= LIMELIGHT.SPHERE_TOLERANCE;
  }

  private static Distance calculateDistance(double targetFOVRatio) {
    Distance absoluteDistance =
        Meters.of((ALGAE.ALGAE_DIAMETER.in(Meters) / 2) / Math.tan(targetFOVRatio));
    return Meters.of(
        Math.sqrt(
            Math.pow(absoluteDistance.in(Meters), 2)
                - Math.pow(
                    Meters.of(LIMELIGHT.ALGAE_CAMERA_POSITION.getY()).in(Meters)
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
