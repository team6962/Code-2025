package frc.robot.vision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

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

public class MegaTag2 extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private SwerveDrivePoseEstimator megaTag1PoseEstimator;

    public MegaTag2(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        megaTag1PoseEstimator = new SwerveDrivePoseEstimator(
            swerveDrive.getKinematics(),
            new Rotation2d(swerveDrive.getContinuousGyroscopeAngle()),
            KinematicsUtils.blankModulePositions(4),
            new Pose2d()
        );
    }

    @Override
    public void periodic() {
        if (CachedRobotState.isDisabled()) seedGyroscopeAngle();

        addVisionEstimates();
    }

    // 1. Get pose estimates from MegaTag1
    // 2. Filter out bad pose estimates
    // 3. Add offset to gyroscope angle

    public void seedGyroscopeAngle() {
        Set<String> cameraIds = LIMELIGHT.APRILTAG_CAMERA_POSES.keySet();

        for (String cameraId : cameraIds) {
            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraId);
            
            for (RawFiducial fiducial : estimate.rawFiducials) {
                Logger.log("Fiducials/" + cameraId + "/" + fiducial.id + "/ambiguity", fiducial.ambiguity);
                Logger.log("Fiducials/" + cameraId + "/" + fiducial.id + "/distToCamera", fiducial.distToCamera);
                Logger.log("Fiducials/" + cameraId + "/" + fiducial.id + "/tagArea", fiducial.ta);
            }

            if (!isEstimateGood(estimate)) continue;

            Logger.getField().getObject("Estimate from " + cameraId).setPose(estimate.pose);

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

        Logger.getField().getObject("Field Relative Heading").setPose(megaTag1PoseEstimator.getEstimatedPosition());

        for (String cameraId : cameraIds) {
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

    public boolean isEstimateGood(PoseEstimate estimate) {
        for (RawFiducial fiducial : estimate.rawFiducials) {
            if (fiducial.ambiguity < 0.4) return true;
        }

        return false;
    }

    // 4. Get pose estimates from MegaTag2
    // 5. Filter out bad pose estimates
    // 6. Send MegaTag2 estimates to swerve

    public void addVisionEstimates() {
        Set<String> cameraIds = LIMELIGHT.APRILTAG_CAMERA_POSES.keySet();

        for (String cameraId : cameraIds) {
            PoseEstimate estimate = CachedRobotState.isBlue().orElse(false) ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraId) :
                LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(cameraId);
            
            if (!isEstimateGood(estimate)) continue;

            Logger.getField().getObject("MegaTag2 estimate from " + cameraId).setPose(estimate.pose);

            swerveDrive.addVisionMeasurement(
                estimate.pose,
                Seconds.of(estimate.timestampSeconds)
            );
        }
    }
}
