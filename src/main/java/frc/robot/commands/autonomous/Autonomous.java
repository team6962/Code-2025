package frc.robot.commands.autonomous;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Field;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.software.MathUtils;

public class Autonomous {
  private RobotStateController controller;
  private SwerveDrive swerveDrive;
  private Manipulator manipulator;
  private Intake intake;
  private Elevator elevator;

  public Autonomous(
      RobotStateController controller,
      SwerveDrive swerveDrive,
      Manipulator manipulator,
      Elevator elevator,
      Intake intake) {
    this.controller = controller;
    this.swerveDrive = swerveDrive;
    this.manipulator = manipulator;
    this.intake = intake;
    this.elevator = elevator;
    
    
    
    // System.out.println(swerveDrive.getEstimatedPose().getX() + ", " + swerveDrive.getEstimatedPose().getY());
    // addCommands(cycleTopCoral());

    // for (int i = 0; i < Field.REEF_FACE_POSITIONS.size(); i ++) {
    //   System.out.println(Meters.convertFrom(Field.REEF_FACE_POSITIONS.get(i).getX(), Inches) + ", " + Meters.convertFrom(Field.REEF_FACE_POSITIONS.get(i).getY(), Inches));
    // }
  }

  public int getClosestReefFace() {
    List<Translation2d> reefFaces = Field.REEF_FACE_POSITIONS;

    double closestDist = Integer.MAX_VALUE;
    int closestFace = -1;

    for (int i = 0; i < reefFaces.size(); i ++) {
      Translation2d curFace = reefFaces.get(i);
      
      double distance = Math.hypot(
        swerveDrive.getEstimatedPose().getX() - curFace.getX(),
        swerveDrive.getEstimatedPose().getY() - curFace.getY()
      );

      System.out.println(distance);

      if (distance < closestDist) {
        closestDist = distance;
        closestFace = i;
      }
    }

    return closestFace;
  }

  /**
   * Outputs the numbers of the reef pole on a certain reef face
   * @param face
   * @return Index 0 is the left pole (from the robot's perspective), index 1 is on the right
   */
  public int[] reefPolesFromReefFace(int face) {
    return new int[] {
      face * 2,
      face * 2 + 1
    };
  }

  public Command pathfindToProcessor() {
    return swerveDrive.pathfindTo(new Pose2d(6.172, 0.508, Rotation2d.fromDegrees(-90)));
  }

  public Command pathfindToTopCoralStation() {
    return swerveDrive.pathfindTo(new Pose2d(1.1, 7.0, Rotation2d.fromDegrees(135)));
  }

  /**
   * Aligns to either reef pole on the closest reef face
   * @param side 0 means left, 1 means right (from the robot's perspective)
   * @return
   */
  public Command reefPoleAlign(int side) {
    return pathfindToReefPole(reefPolesFromReefFace(getClosestReefFace())[side]);
  }

  public Command pathfindToTopLeftReefPoles() {
    return swerveDrive.pathfindTo(new Pose2d(3.45, 5.4, Rotation2d.fromDegrees(300)));
  }

  /**
   * Pathfinds to reef pole based on pole number
   * @param poleNum Number from 1-12 starting on the right side top pole, moving counterclockwise
   * @return
   */
  public Command pathfindToReefPole(int poleNum) {
    return swerveDrive.pathfindTo(new Pose2d(
      Field.CORAL_PLACEMENT_POSES.get(poleNum).getX(),
      Field.CORAL_PLACEMENT_POSES.get(poleNum).getY(),
      Field.CORAL_PLACEMENT_POSES.get(poleNum).getRotation()
    ));
  }

  public Command scoreCoral() {
    return null;
  }

  public Command cycleTopCoral() {
    return Commands.sequence(
      pathfindToReefPole(4),
      pathfindToTopCoralStation(),
      pathfindToReefPole(3),
      pathfindToTopCoralStation(),
      pathfindToReefPole(2),
      pathfindToTopCoralStation(),
      pathfindToReefPole(1),
      pathfindToTopCoralStation(),
      pathfindToReefPole(0),
      pathfindToTopCoralStation()
    );
  }

  public Command coralStation() {
    return null;
  }

  private static Translation2d ALGAE_SETUP = new Translation2d(1.98, 2.18);
  private static Translation2d ALGAE_DRIVE_OVER = new Translation2d(1.21, 2.18);

  public static enum AlgaePickupMechanism {
    INTAKE,
    MANIPULATOR
  }

  /**
   * Pickup a pre-placed algae on the ground
   *
   * @param algaePosition The position of the algae to pickup (0 is nearest the processor, and 2 is
   *     farthest)
   * @return
   */
  public Command pickupPreplacedAlgae(int algaePosition, AlgaePickupMechanism mechanism) {
    Translation2d offset = new Translation2d(0, 1.83 * algaePosition);
    Rotation2d angle =
        Rotation2d.fromDegrees(mechanism == AlgaePickupMechanism.MANIPULATOR ? -180 : 0);

    Command setupCommand =
        swerveDrive.pathfindTo(
            new Pose2d(ALGAE_SETUP.plus(offset), angle));
    Command driveOverCommand =
        swerveDrive.pathfindTo(new Pose2d(ALGAE_DRIVE_OVER.plus(offset), angle));

    return Commands.sequence(
      setupCommand,
      Commands.deadline(
        Commands.sequence(
          driveOverCommand,
          intake.pivot.lower()
        ),
        intake.wheels.intake()
      ),
      intake.pivot.raise()
    );

    // if (mechanism == AlgaePickupMechanism.INTAKE) {
    //   return Commands.sequence(
    //       setupCommand,
    //       Commands.deadline(driveOverCommand, intake.wheels.intake(), intake.pivot.lower()),
    //       intake.pivot.raise());
    // } else {
    //   
    // }
  }
  
  final double BARGE_X = 7.75;
  final double BARGE_MIN = 4.63;
  final double BARGE_MAX = 7.41;

  public Command driveToBarge() {
    double y = swerveDrive.getEstimatedPose().getY();

    y = MathUtil.clamp(y, BARGE_MIN, BARGE_MAX);

    return swerveDrive.pathfindTo(new Pose2d(BARGE_X, y, Rotation2d.fromDegrees(180)));
  }

  public boolean hasCoral() {
    return false;
  }

  public Command createAutonomousCommand() {
    return Commands.sequence(
      reefPoleAlign(1)
    );
  }
}
