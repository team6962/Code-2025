package frc.robot.vision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.Map;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.LIMELIGHT;
import frc.robot.util.CachedRobotState;
import io.limelightvision.LimelightHelpers;
import io.limelightvision.LimelightHelpers.PoseEstimate;
import io.limelightvision.LimelightHelpers.RawFiducial;

/**
 * The MegaTag2 class is responsible for pose estimation using a combination
 * of MegaTag1 and MegaTag2.
 */
public class MegaTag2 extends SubsystemBase {
    private SwerveDrive swerveDrive;

    /**
     * Pose estimator used to find field-relative headings with MegaTag1.
     */
    private SwerveDrivePoseEstimator megaTag1PoseEstimator;

    /**
     * Map from limelight ids to most recent MegaTag2-generated pose estimates,
     * used for logging.
     */
    private Map<String, Pose2d> megaTag2PoseEstimates = new HashMap<>(LIMELIGHT.APRILTAG_CAMERA_IDS.size());

    public MegaTag2(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        megaTag1PoseEstimator = new SwerveDrivePoseEstimator(
            swerveDrive.getKinematics(),
            new Rotation2d(swerveDrive.getContinuousGyroscopeAngle()),
            KinematicsUtils.blankModulePositions(4),
            new Pose2d()
        );

        Logger.addUpdate("poseEstimationData", this::logPoseEstimationData);
    }

    @Override
    public void periodic() {
        if (CachedRobotState.isDisabled()) seedGyroscopeAngle();

        addVisionEstimates();
    }

    private void logPoseEstimationData() {
        for (String cameraId : LIMELIGHT.APRILTAG_CAMERA_IDS) {
            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraId);

            Logger.log("PoseEstimation/" + cameraId + "/pose", estimate.pose);
            Logger.log("PoseEstimation/" + cameraId + "/latency", estimate.latency);
            Logger.log("PoseEstimation/" + cameraId + "/tagCount", estimate.tagCount);
            Logger.log("PoseEstimation/" + cameraId + "/timestampSeconds", estimate.timestampSeconds);

            for (RawFiducial fiducial : estimate.rawFiducials) {
                Logger.log("PoseEstimation/" + cameraId + "/tags/" + fiducial.id + "/ambiguity", fiducial.ambiguity);
                Logger.log("PoseEstimation/" + cameraId + "/tags/" + fiducial.id + "/distToCamera", fiducial.distToCamera);
                Logger.log("PoseEstimation/" + cameraId + "/tags/" + fiducial.id + "/tagArea", fiducial.ta);
            }

            Logger.log("PoseEstimation/" + cameraId + "/accurate", isEstimateAccurate(estimate));
        }
    }

    /**
     * Gives MegaTag2 field-relative gyroscope angles detected by MegaTag1.
     */
    public void seedGyroscopeAngle() {
        for (String cameraId : LIMELIGHT.APRILTAG_CAMERA_IDS) {
            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraId);
            
            if (!isEstimateAccurate(estimate)) continue;

            megaTag1PoseEstimator.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds
            );
        }

        megaTag1PoseEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            new Rotation2d(swerveDrive.getContinuousGyroscopeAngle()),
            KinematicsUtils.blankModulePositions(4)
        );

        Rotation2d fieldRelativeHeading = megaTag1PoseEstimator.getEstimatedPosition().getRotation();

        for (String cameraId : LIMELIGHT.APRILTAG_CAMERA_IDS) {
            LimelightHelpers.SetRobotOrientation(
                cameraId,
                fieldRelativeHeading.getDegrees(),
                0,
                0,
                0,
                0,
                0
            );
        }
    }

    /**
     * Checks if a pose estimate is roughly accurate.
     * @param estimate The pose estimate to check the accuracy of.
     * @return True if it's somewhat accurate.
     */
    public boolean isEstimateAccurate(PoseEstimate estimate) {
        for (RawFiducial fiducial : estimate.rawFiducials) {
            if (fiducial.ambiguity < 0.4) return true;
        }

        return false;
    }

    /**
     * Adds vision measurements from MegaTag2 to the swerve drive pose
     * estimator.
     */
    public void addVisionEstimates() {
        for (String cameraId : LIMELIGHT.APRILTAG_CAMERA_IDS) {
            PoseEstimate estimate = CachedRobotState.isBlue().orElse(false) ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraId) :
                LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(cameraId);
            
            if (!isEstimateAccurate(estimate)) continue;

            megaTag2PoseEstimates.put(cameraId, estimate.pose);

            swerveDrive.addVisionMeasurement(
                estimate.pose,
                Seconds.of(estimate.timestampSeconds)
            );
        }
    }
}
