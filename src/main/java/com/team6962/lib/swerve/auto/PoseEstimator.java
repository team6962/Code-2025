package com.team6962.lib.swerve.auto;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * {@code PoseEstimator} is a class that estimates the pose of a swerve drive
 * robot using a combination of odometry, gyroscope, and vision measurements.
 * A PoseEstimator contains a {@link SwerveDrivePoseEstimator} that is updated
 * with the latest measurements. It also contains a {@link SwerveGyroscope} that
 * can be used to get and reset the robot's heading.
 * <p>
 * Vision data can be added with {@link #addVisionMeasurement(Pose2d, Time)}.
 */
public class PoseEstimator extends SubsystemBase {
    private SwerveDriveKinematics kinematics;
    private SwerveGyroscope gyroscope;
    private SwerveDrivePoseEstimator poseEstimator;

    private Supplier<SwerveModulePosition[]> modulePositionsSupplier;
    private Supplier<SwerveModuleState[]> moduleStatesSupplier;

    private SwerveModulePosition[] lastPositions;
    private SwerveModulePosition[] positionChanges = new SwerveModulePosition[4];
    private Twist2d chassisVelocity = new Twist2d();

    public PoseEstimator(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> modulePositions, Supplier<SwerveModuleState[]> moduleStates) {
        this.kinematics = kinematics;
        this.gyroscope = new SwerveGyroscope(() -> positionChanges, kinematics);
        this.modulePositionsSupplier = modulePositions;
        this.moduleStatesSupplier = moduleStates;

        lastPositions = modulePositions.get();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroscope.getHeading(), modulePositions.get(), new Pose2d());
    }

    @Override
    public void periodic() {
        SwerveModulePosition[] modulePositions = this.modulePositionsSupplier.get();
        Time timestamp = Seconds.of(Timer.getFPGATimestamp());

        poseEstimator.updateWithTime(timestamp.in(Seconds), gyroscope.getHeading(), modulePositions);
        
        chassisVelocity = kinematics.toTwist2d(KinematicsUtils.toModulePositions(moduleStatesSupplier.get(), Seconds.of(1)));

        positionChanges = KinematicsUtils.difference(modulePositions, lastPositions);
    }

    public SwerveGyroscope getGyroscope() {
        return gyroscope;
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, Time timestamp) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestamp.in(Seconds));
    }

    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        Time timestamp,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestamp.in(Seconds), visionMeasurementStdDevs);
    }

    public void resetPosition(Pose2d expectedPose) {
        poseEstimator.resetPosition(gyroscope.getHeading(), lastPositions, expectedPose);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getFuturePose(Time time) {
        Pose2d currentPose = getEstimatedPose();
        
        if (chassisVelocity == null) return currentPose;

        double deltaSeconds = time.in(Seconds);
        Twist2d deltaTwist = new Twist2d(chassisVelocity.dx * deltaSeconds, chassisVelocity.dy * deltaSeconds, chassisVelocity.dtheta * deltaSeconds);

        return currentPose.exp(deltaTwist);
    }

    public Pose2d getEstimatedPose(Time timestamp) {
        return poseEstimator.sampleAt(timestamp.in(Seconds)).orElseGet(this::getEstimatedPose);
    }

    public ChassisSpeeds getEstimatedSpeeds() {
        return kinematics.toChassisSpeeds(moduleStatesSupplier.get());
    }
}
