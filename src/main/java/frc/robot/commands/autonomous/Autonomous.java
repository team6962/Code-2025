package frc.robot.commands.autonomous;

import java.util.Set;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.commands.PieceCombos;
import frc.robot.Constants.ReefPositioning;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
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

    // System.out.println(swerveDrive.getEstimatedPose().getX() + ", " +
    // swerveDrive.getEstimatedPose().getY());
    // addCommands(cycleTopCoral());

    // for (int i = 0; i < Field.REEF_FACE_POSITIONS.size(); i ++) {
    //   System.out.println(Meters.convertFrom(Field.REEF_FACE_POSITIONS.get(i).getX(), Inches) + ",
    // " + Meters.convertFrom(Field.REEF_FACE_POSITIONS.get(i).getY(), Inches));
    // }
  }

  // public int getClosestReefFace() {
  //   List<Translation2d> reefFaces = Field.REEF_FACE_POSITIONS;

  //   double closestDist = Integer.MAX_VALUE;
  //   int closestFace = -1;

  //   for (int i = 0; i < reefFaces.size(); i ++) {
  //     Translation2d curFace = reefFaces.get(i);
      
  //     double distance = Math.hypot(
  //       swerveDrive.getEstimatedPose().getX() - curFace.getX(),
  //       swerveDrive.getEstimatedPose().getY() - curFace.getY()
  //     );

  //     System.out.println(distance);

  //     if (distance < closestDist) {
  //       closestDist = distance;
  //       closestFace = i;
  //     }
  //   }

  //   return closestFace;
  // }

  /**
   * Outputs the numbers of the reef pole on a certain reef face
   *
   * @param face
   * @return Index 0 is the left pole (from the robot's perspective), index 1 is on the right
   */
  public int[] reefPolesFromReefFace(int face) {
    return new int[] {face * 2, face * 2 + 1};
  }

  public Command driveToProcessor() {
    return swerveDrive.pathfindTo(new Pose2d(6.172, 0.508, Rotation2d.fromDegrees(-90)));
  }

  // public Command pathfindToTopCoralStation() {
  //   return swerveDrive.pathfindTo(new Pose2d(1.1, 7.0, Rotation2d.fromDegrees(135)));
  // }

  // /**
  //  * Aligns to either reef pole on the closest reef face
  //  * @param side 0 means left, 1 means right (from the robot's perspective)
  //  * @return
  //  */
  // public Command reefPoleAlign(int side) {
  //   return pathfindToReefPole(reefPolesFromReefFace(getClosestReefFace())[side]);
  // }

  // public Command pathfindToTopLeftReefPoles() {
  //   return swerveDrive.pathfindTo(new Pose2d(3.45, 5.4, Rotation2d.fromDegrees(300)));
  // }

  // /**
  //  * Pathfinds to reef pole based on pole number
  //  *
  //  * @param poleNum Number from 1-12 starting on the right side top pole, moving counterclockwise
  //  * @return
  //  */
  // public Command pathfindToReefPole(int poleNum) {
  //   return swerveDrive.pathfindTo(
  //       new Pose2d(
  //           Meters.convertFrom(Field.CORAL_PLACEMENT_POSES.get(poleNum).getX(), Inches),
  //           Meters.convertFrom(Field.CORAL_PLACEMENT_POSES.get(poleNum).getY(), Inches),
  //           Field.CORAL_PLACEMENT_POSES.get(poleNum).getRotation()));
  // }

  // public Command pathfindToReefPole(int faceNumber, Pole pole) {
  //   Pose2d polePose = Field.getPolePose(faceNumber, pole);
  //   return swerveDrive.pathfindTo(polePose);
  // }

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

  // /**
  //  * Aligns to either reef pole on the closest reef face
  //  *
  //  * @param side 0 means left, 1 means right (from the robot's perspective)
  //  * @return
  //  */
  // public Command reefPoleAlign(int side) {
  //   return pathfindToReefPole(reefPolesFromReefFace(getClosestReefFace())[side]);
  // }

  public Command driveToAlgae(int face) {
    return swerveDrive.pathfindTo(ReefPositioning.getAlgaeAlignPose(face))
      .andThen(swerveDrive.alignTo(ReefPositioning.getAlgaePlacePose(face)));
  }

  // public Command scoreCoral() {
  //   return null;
  // }

  // public Command cycleTopCoral() {
  //   return Commands.sequence(
  //     pathfindToReefPole(4),
  //     pathfindToTopCoralStation(),
  //     pathfindToReefPole(3),
  //     pathfindToTopCoralStation(),
  //     pathfindToReefPole(2),
  //     pathfindToTopCoralStation(),
  //     pathfindToReefPole(1),
  //     pathfindToTopCoralStation(),
  //     pathfindToReefPole(0),
  //     pathfindToTopCoralStation()
  //   );
  // }

  // public Command coralStation() {
  //   return null;
  // }

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
      driveToAlgae(3),
      driveToBarge(),
      placeCoral(11, 4),
      driveToAlgae(5),
      driveToBarge()
    );
    // return Commands.sequence(
    //   reefPoleAlign(1)
    // );
  }
}
