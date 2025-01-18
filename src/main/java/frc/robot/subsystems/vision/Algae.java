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
import frc.robot.Constants.Field;
import frc.robot.subsystems.LEDs;

public class Algae {
    public static Translation2d getAlgaePosition(String name, Rotation2d pitch, SwerveDrive swerveDrive, Translation2d fieldVelocity, Translation3d cameraToRobot){
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);

        if (table.getEntry("tv").getDouble(0) == 0) return null;

        Translation2d algaePosition = new Translation2d();
        
        double x = table.getEntry("tx").getDouble(0);
        
        double[] t2d = NetworkTableInstance.getDefault().getTable("limelight").getEntry("t2d").getDoubleArray(new double[]{});

        double pixelHeight = t2d[15];
        double pixelWidth = t2d[14];
        
        // Validate the object is approximately spherical
        if (Math.abs(pixelHeight - pixelWidth) > 5) return null; // Not spherical (tolerance of 5 pixels)
        
        // Calculate distance using apparent size
        double distance = (ALGAE.ALGAE_DIAMETER / 2) / Math.tan(
            LIMELIGHT.FOV_HEIGHT.getRadians() * (pixelHeight / LIMELIGHT.ALGAE_CAMERA_HEIGHT_PIXELS)
        );
        
        // Calculate relative position
        Translation2d relativePosition = new Translation2d(
            distance * Math.cos(Units.degreesToRadians(x)),
            -distance * Math.sin(Units.degreesToRadians(x))
        ).plus(cameraToRobot.toTranslation2d());

        double latency = (table.getEntry("tl").getDouble(0) + table.getEntry("cl").getDouble(0));  //You have to add these to get total latency
        double timestamp = (table.getEntry("hb").getLastChange() / 1000000.0) - (latency / 1000.0);

        Pose2d robotPosition = swerveDrive.getEstimatedPose(Seconds.of(timestamp));
        algaePosition = robotPosition.getTranslation().plus(relativePosition.rotateBy(robotPosition.getRotation()));
        
        if (!RobotState.isDisabled()) {
            LEDs.setState(LEDs.State.CAN_SEE_NOTE);
          }
          
        return algaePosition;
    }
}
