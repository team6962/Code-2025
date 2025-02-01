package com.team6962.lib.swerve;

import com.ctre.phoenix6.Orchestra;
import com.team6962.lib.swerve.module.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that plays a .chrp midi file in src/main/deploy/music on on the swerve drive's Kraken
 * X60s
 */
public class MusicDrive extends Command {
  private Orchestra orchestra;

  public MusicDrive(SwerveDrive swerveDrive, String name) {
    orchestra = new Orchestra();

    for (SwerveModule module : swerveDrive.getModules()) {
      orchestra.addInstrument(module.getDriveMotor());
      orchestra.addInstrument(module.getSteerMotor());
    }

    orchestra.loadMusic("music/" + name + ".chrp");

    addRequirements(swerveDrive.useMotion());
  }

  @Override
  public void initialize() {
    orchestra.play();
  }

  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
  }
}
