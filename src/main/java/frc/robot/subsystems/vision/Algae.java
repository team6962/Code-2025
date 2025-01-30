package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Constants.ALGAE;
import frc.robot.subsystems.LEDs;
import io.limelightvision.LimelightHelpers;

public class Algae {
    public static Translation2d getAlgaePosition(String name, Rotation2d pitch, SwerveDrive swerveDrive, Translation3d cameraToRobot){
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);

        if (table.getEntry("tv").getDouble(0) == 0) return null; //check if detection is valid

        if (!table.getEntry("tdclass").getString("").equals("algae")) return null;
        //System.out.println("EIIMA CRAHSIGN OUT");
        Translation2d algaePosition = new Translation2d(0,0);
        
        double horizontalOffset = table.getEntry("tx").getDouble(0); //the horizontal offset angle of target from center of camera
        
        double[] t2d = LimelightHelpers.getT2DArray(name);
        System.out.println(t2d[16]);
        /*double pixelHeight = t2d[15];
        double pixelRatio = pixelHeight/t2d[14];
        System.out.println("wawiefawpeiofjapweiofjawfea" + t2d);
        // Validate the object is approximately spherical
        if ((pixelRatio < 1.0 + LIMELIGHT.SPHERE_TOLERANCE) && (pixelRatio > 1.0 - LIMELIGHT.SPHERE_TOLERANCE) && (pixelHeight > 0)) return null; // Not spherical
        System.out.println("fovhegith" + LIMELIGHT.FOV_HEIGHT.getRadians());
        System.out.println("pixelHieght" + pixelHeight);
        double targetFOVRatio = LIMELIGHT.FOV_HEIGHT.getRadians() * (pixelHeight/ LIMELIGHT.ALGAE_CAMERA_HEIGHT_PIXELS);//portion of height target occupies in radians
        if (targetFOVRatio > Math.PI/2) return null;
        System.out.println("targetFOVRATION" + targetFOVRatio);
        // Calculate distance using apparent size
        double distance = (ALGAE.ALGAE_DIAMETER / 2) / Math.tan(targetFOVRatio);
        //if (distance > LIMELIGHT.MAX_DETECTION_RANGE) return null;
        // Calculate relative position
        System.out.println("debugger");
        System.out.println("distance:" + distance);
        Translation2d relativePosition = new Translation2d(
            distance * Math.cos(Units.degreesToRadians(horizontalOffset)),
            -distance * Math.sin(Units.degreesToRadians(horizontalOffset))
        ).plus(cameraToRobot.toTranslation2d());

        double latency = (table.getEntry("tl").getDouble(0) + table.getEntry("cl").getDouble(0));  //You have to add these to get total latency
        double timestamp = (table.getEntry("hb").getLastChange() / 1000000.0) - (latency / 1000.0);

        Pose2d robotPosition = swerveDrive.getEstimatedPose(Seconds.of(timestamp));
        algaePosition = robotPosition.getTranslation().plus(relativePosition.rotateBy(robotPosition.getRotation()));
        
        if (!RobotState.isDisabled()) {
            LEDs.setState(LEDs.State.CAN_SEE_ALGAE);
          }
*/
        return algaePosition;
        
    }
}
