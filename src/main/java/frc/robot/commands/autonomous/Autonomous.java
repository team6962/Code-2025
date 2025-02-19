package frc.robot.commands.autonomous;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Constants.SWERVE;
import frc.robot.Constants.ReefPositioning;
import frc.robot.commands.PieceCombos;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Algae;

public class Autonomous {
  private RobotStateController controller;
  private SwerveDrive swerveDrive;
  private Manipulator manipulator;
  private Elevator elevator;
  private PieceCombos pieceCombos;

  public Autonomous(
      RobotStateController controller,
      SwerveDrive swerveDrive,
      Manipulator manipulator,
      Elevator elevator,
      PieceCombos pieceCombos
    ) {
    this.controller = controller;
    this.swerveDrive = swerveDrive;
    this.manipulator = manipulator;
    this.elevator = elevator;
    this.pieceCombos = pieceCombos;
  }

  public int getClosestReefPole(Pose2d pose) {
    int closestPole = 0;
    double closestDistance = Double.MAX_VALUE;

    for (int i = 0; i < 12; i++) {
      Translation2d pole = ReefPositioning.getCoralAlignPose(i).getTranslation();

      if (pose.getTranslation().getDistance(pole) < closestDistance) {
        closestPole = i;
        closestDistance = pose.getTranslation().getDistance(pole);
      }
    }

    return closestPole;
  }

  public Command processAlgae() {
    return Commands.sequence(
      swerveDrive.pathfindTo(new Pose2d(
        Units.inchesToMeters(235.726104),
        SWERVE.CONFIG.chassis().outerWidth().div(2).plus(Inches.of(18)).in(Meters),
        Rotation2d.fromDegrees(90)
      )),
      CommandUtils.selectByMode(pieceCombos.algaeProcessor(), Commands.print("Preparing for processor")),
      swerveDrive.alignTo(new Pose2d(
        Units.inchesToMeters(235.726104),
        SWERVE.CONFIG.chassis().outerWidth().div(2).plus(Inches.of(14)).in(Meters),
        Rotation2d.fromDegrees(90)
      )),
      CommandUtils.printInSimulation("Dropping algae"),
      manipulator.algae.drop(),
      swerveDrive.pathfindTo(new Pose2d(
        Units.inchesToMeters(235.726104),
        SWERVE.CONFIG.chassis().outerWidth().div(2).plus(Inches.of(18)).in(Meters),
        Rotation2d.fromDegrees(90)
      ))
    );
  }

  public Command driveToPole(int pole) {
    return swerveDrive.pathfindTo(ReefPositioning.getCoralAlignPose(pole))
      .andThen(swerveDrive.alignTo(ReefPositioning.getCoralPlacePose(pole)));
  }

  public Command placeCoral(int pole, int level) {
    return Commands.sequence(
      swerveDrive.pathfindTo(ReefPositioning.getCoralAlignPose(pole)),
      CommandUtils.selectByMode(pieceCombos.coral(level), CommandUtils.logAndWait("Moving to level " + level, level * 0.5)),
      swerveDrive.alignTo(ReefPositioning.getCoralPlacePose(pole)),
      CommandUtils.selectByMode(manipulator.coral.drop(), CommandUtils.logAndWait("Dropping coral", 0.25)),
      CommandUtils.selectByMode(pieceCombos.stow(), CommandUtils.logAndWait("Stowing elevator", level * 0.5))
    );
  }

  public Command driveToAlgae(int face) {
    return swerveDrive.pathfindTo(ReefPositioning.getAlgaeAlignPose(face))
      .andThen(swerveDrive.alignTo(ReefPositioning.getAlgaePlacePose(face)));
  }

  public Command pickupAlgae(int face, int level) {
    return Commands.sequence(
      swerveDrive.pathfindTo(ReefPositioning.getAlgaeAlignPose(face)),
      CommandUtils.selectByMode(pieceCombos.algae(level), CommandUtils.logAndWait("Moving to level " + level, level * 0.5)),
      swerveDrive.alignTo(ReefPositioning.getAlgaePlacePose(face)),
      CommandUtils.selectByMode(manipulator.algae.intake(), CommandUtils.logAndWait("Intaking algae", 0.5)),
      swerveDrive.alignTo(ReefPositioning.getAlgaeLeavePose(face), Inches.of(3), Degrees.of(45)),
      CommandUtils.selectByMode(pieceCombos.stow(), CommandUtils.logAndWait("Stowing elevator", level * 0.5))
    );
  }

  public Command pathfindToPole(int poleNumber) {
    return swerveDrive
        .pathfindTo(ReefPositioning.getCoralAlignPose(poleNumber))
        .andThen(swerveDrive.alignTo(ReefPositioning.getCoralPlacePose(poleNumber)));
  }

  // private static Translation2d ALGAE_SETUP = new Translation2d(1.98, 2.18);
  // private static Translation2d ALGAE_DRIVE_OVER = new Translation2d(1.21, 2.18);

  final double BARGE_X = 7.75;
  final double BARGE_MIN = 4.63;
  final double BARGE_MAX = 7.41;

  public Command driveToBarge() {
    return Commands.defer(() -> {
      double y = swerveDrive.getEstimatedPose().getY();

      y = MathUtil.clamp(y, BARGE_MIN, BARGE_MAX);

      Pose2d pose = new Pose2d(BARGE_X, y, Rotation2d.fromDegrees(180));

      return swerveDrive.pathfindTo(pose).andThen(swerveDrive.alignTo(pose));
    }, Set.of(swerveDrive.useMotion()));
  }

  public boolean hasCoral() {
    return false;
  }

  public Command autoOrientToAlgae() {
    return swerveDrive.facePoint(
        () ->
            Algae.getAlgaePosition(
                LIMELIGHT.ALGAE_CAMERA_NAME, swerveDrive, LIMELIGHT.ALGAE_CAMERA_POSITION));
}

  public Command createAutonomousCommand() {
    return Commands.sequence(
      placeCoral(0, 1),
      pickupAlgae(3, 3),
      processAlgae(),
      placeCoral(11, 4),
      pickupAlgae(5, 2),
      driveToBarge()
    );
  }
}
