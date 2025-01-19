package com.team6962.lib.test;

import java.lang.reflect.Array;
import java.util.Arrays;

import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.module.SimulatedModule;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KinematicsTest extends SubsystemBase implements AutoCloseable {
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private Pose2d robotPose = new Pose2d();
    private SwerveModulePosition[] oldPositions;

    public KinematicsTest(SwerveConfig config) {
        modules = new SwerveModule[4];
        oldPositions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            @SuppressWarnings("resource")
            SwerveModule module = new SimulatedModule();
            module.configureModule(config, SwerveModule.Corner.fromIndex(i));

            modules[i] = module;
            oldPositions[i] = new SwerveModulePosition();
        }

        kinematics = KinematicsUtils.kinematicsFromChassis(config.chassis());
    }

    @Override
    public void periodic() {
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(0, 0, Math.PI / 2);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(targetSpeeds);

        for (int i = 0; i < 4; i++) {
            modules[i].driveState(moduleStates[i]);
        }

        SwerveModulePosition[] newPositions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            newPositions[i] = modules[i].getPosition();
        }

        SwerveModulePosition[] positionDeltas = KinematicsUtils.difference(newPositions, oldPositions);
        // SwerveModulePosition[] expectedDeltas = Arrays.stream(moduleStates).map(state -> new SwerveModulePosition(
        //     state.speedMetersPerSecond * 0.02,
        //     state.angle.times(0.02)
        // )).toArray(SwerveModulePosition[]::new);

        Logger.log("KTest/MeasuredDeltas", positionDeltas);
        // Logger.log("KTest/ExpectedDeltas", expectedDeltas);

        robotPose = robotPose.exp(kinematics.toTwist2d(positionDeltas));

        Logger.getField().getObject("TestBot").setPose(robotPose);

        oldPositions = newPositions;
    }

    @Override
    public void close() throws Exception {
        for (SwerveModule module : modules) {
            module.close();
        }
    }
}
