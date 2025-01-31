package com.team6962.lib.swerve;

import java.util.Arrays;

import com.team6962.lib.swerve.auto.Coordinates;
import com.team6962.lib.swerve.auto.PoseEstimator;
import com.team6962.lib.swerve.auto.SwerveGyroscope;
import com.team6962.lib.swerve.module.SimulatedModule;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * The "core" of the swerve drive system. This class is responsible for managing
 * the swerve modules, the pose estimator, and the current movement the drivetrain
 * is performing. {@code SwerveCore} is extended by {@link SwerveDrive} to provide
 * the {@link Command}-based API for controlling the drivetrain.
 */
public class SwerveCore extends SubsystemBase implements Coordinates {
    private PoseEstimator poseEstimator;
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveConfig constants;

    private SwerveMovement currentMovement;

    public SwerveCore(SwerveConfig constants) {
        this.constants = constants;

        modules = new SwerveModule[4];

        for (int i = 0; i < 4; i++) {
            SwerveModule module = RobotBase.isReal() ? new SwerveModule() : new SimulatedModule();
            module.configureModule(constants, SwerveModule.Corner.fromIndex(i));

            modules[i] = module;
        }

        kinematics = KinematicsUtils.kinematicsFromChassis(constants.chassis());
        poseEstimator = new PoseEstimator(kinematics, () -> getModulePositions(), () -> getModuleStates());

        currentMovement = new SwerveMovement(kinematics);

        Logger.logPose("Swerve Drive/robotPose", poseEstimator::getEstimatedPose);
    }

    public SwerveConfig getConstants() {
        return constants;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public PoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public SwerveGyroscope getGyroscope() {
        return poseEstimator.getGyroscope();
    }

    @Override
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPose();
    }

    public Pose2d getEstimatedPose(Time timestamp) {
        return poseEstimator.getEstimatedPose(timestamp);
    }

    public Pose2d[] getModulePoses() {
        Pose2d[] poses = new Pose2d[4];

        for (int i = 0; i < 4; i++) {
            Transform2d relativeTransform = modules[i].getRelativeTransform();

            poses[i] = poseEstimator.getEstimatedPose().transformBy(relativeTransform);
        }

        return poses;
    }

    /**
     * Should be called after CommandScheduler.run() to mimimize latency.
     */
    public void latePeriodic() {
        SwerveModuleState[] states = currentMovement.getStates();
        
        if (states == null) {
            states = KinematicsUtils.getStoppedStates(getModuleStates());
        }

        states = KinematicsUtils.desaturateWheelSpeeds(states, constants.maxDriveSpeed());

        Logger.log("Drivetrain/targetModuleSpeeds", robotToAllianceSpeeds(kinematics.toChassisSpeeds(states)));
        Logger.log("Drivetrain/targetModuleSpeeds_robotRelative", kinematics.toChassisSpeeds(states));
        Logger.log("Drivetrain/targetModuleStates", states);
        Logger.log("Drivetrain/currentModuleStates", getModuleStates());

        // states = new SwerveModuleState[] {
        //     new SwerveModuleState(-2, Rotation2d.fromDegrees(0)),
        //     new SwerveModuleState(-2, Rotation2d.fromDegrees(0)),
        //     new SwerveModuleState(-2, Rotation2d.fromDegrees(0)),
        //     new SwerveModuleState(-2, Rotation2d.fromDegrees(0))
        // };

        Pose2d[] poses = getModulePoses();

        for (int i = 0; i < 4; i++) {
            modules[i].driveState(new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle));
            poses[i] = new Pose2d(poses[i].getTranslation(), poseEstimator.getEstimatedPose().getRotation().plus(states[i].angle));
        }

        Logger.getField().getObject("Target Module Poses").setPoses(poses);

        currentMovement.clear();
    }

    public void setMovement(SwerveModuleState[] states) {
        currentMovement.setStates(states);
    }

    public void setMovement(ChassisSpeeds speeds) {
        Logger.log("Swerve Drive/motionSpeeds", RobotContainer.getInstance().swerveDrive.robotToAllianceSpeeds(speeds));

        currentMovement.setSpeeds(speeds);
    }

    public void setMovement(Translation2d translation, Rotation2d rotation) {
        currentMovement.setSpeeds(KinematicsUtils.createChassisSpeeds(translation, rotation));
    }

    public void setMovement(Translation2d translation) {
        currentMovement.setTranslation(translation);
    }

    public void setMovement(Rotation2d rotation) {
        currentMovement.setRotation(rotation);
    }
}
