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
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.Constants.ALGAE;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.subsystems.LEDs;

public class Algae {
    public static Translation2d getAlgaePosition(String name, SwerveDrive swerveDrive, Translation3d cameraToRobot){
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);

    if (table.getEntry("tv").getDouble(0) == 0) return null; // check if detection is valid

        if (!table.getEntry("tdclass").getString("").equals("algae")) return null;

        Translation2d algaePosition = new Translation2d(0,0);
        
        Angle horizontalOffset = Degrees.of(table.getEntry("tx").getDouble(0)); //the horizontal offset angle of target from center of camera
        double[] t2d = table.getEntry("t2d").getDoubleArray(new double[17]);
        double pixelHeight = t2d[15];
        double pixelRatio = pixelHeight/t2d[14];

        // Validate the object is approximately spherical
        if (!(pixelRatio < 1.0 + LIMELIGHT.SPHERE_TOLERANCE) && !(pixelRatio > 1.0 - LIMELIGHT.SPHERE_TOLERANCE) && !(pixelHeight > 0)) return null; // Not spherical
        double targetFOVRatio = LIMELIGHT.FOV_HEIGHT.getRadians() * (pixelHeight/ LIMELIGHT.ALGAE_CAMERA_HEIGHT_PIXELS);//portion of height target occupies in radians
        if (targetFOVRatio > Math.PI/2) return null;
        // Calculate distance using apparent size
        Distance absolute_distance = Meters.of((ALGAE.ALGAE_DIAMETER.in(Meters) / 2) / Math.tan(targetFOVRatio));
        // Use pythagorean theorem to account for limelight's height
        Distance distance = Meters.of(Math.sqrt(Math.pow(absolute_distance.in(Meters),2)-Math.pow(Meters.of(LIMELIGHT.ALGAE_CAMERA_POSITION.getY()).in(Meters) - (ALGAE.ALGAE_DIAMETER.in(Meters))/2,2)));
        
        //Check if the detected algae is actually in the field
        if (distance.gt(LIMELIGHT.MAX_DETECTION_RANGE)) return null;
        
        // Calculate relative position
        Translation2d relativePosition = new Translation2d(
            distance.in(Meters) * Math.cos(horizontalOffset.in(Radians)),
            -distance.in(Meters) * Math.sin(horizontalOffset.in(Radians))
        ).plus(cameraToRobot.toTranslation2d());

    double latency =
        (table.getEntry("tl").getDouble(0)
            + table.getEntry("cl").getDouble(0)); // You have to add these to get total latency
    double timestamp = (table.getEntry("hb").getLastChange() / 1000000.0) - (latency / 1000.0);

    Pose2d robotPosition = swerveDrive.getEstimatedPose(Seconds.of(timestamp));
    algaePosition =
        robotPosition.getTranslation().plus(relativePosition.rotateBy(robotPosition.getRotation()));

    if (!RobotState.isDisabled()) {
      LEDs.setState(LEDs.State.CAN_SEE_ALGAE);
    }

    return algaePosition;
  }
}
