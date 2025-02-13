package com.team6962.lib.swerve.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoBuilderWrapper {
  private static boolean instantiated;

  private Consumer<ChassisSpeeds> output;

  public AutoBuilderWrapper() {
    if (instantiated) {
      throw new RuntimeException("AutoBuilderWrapper is a singleton!");
    }

    instantiated = true;

    FollowPathCommand.warmupCommand().schedule();
  }

  public void setOutput(Consumer<ChassisSpeeds> output) {
    this.output = output;
  }

  public void configure(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> speedsSupplier,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath) {
    AutoBuilder.configure(
        poseSupplier,
        resetPose,
        speedsSupplier,
        t -> output.accept(t),
        controller,
        robotConfig,
        shouldFlipPath);
  }
}
