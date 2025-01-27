package frc.robot.subsystems.manipulator;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.MotionControl.PivotController;

public class ManipulatorPivot extends SubsystemBase{
    
    private SparkMax motor;
    private PivotController controller;
    private boolean isCalibrating = false;
    
    public ManipulatorPivot(){
        motor = new SparkMax(CAN.SHOOTER_PIVOT, MotorType.kBrushless);
        controller = new PivotController(
            this,
            motor,
            DIO.MANIPULATOR_PIVOT,
            MANIPULATOR_PIVOT.ABSOLUTE_POSITION_OFFSET,
            MANIPULATOR_PIVOT.PROFILE.kP,
            MANIPULATOR_PIVOT.PROFILE.kS,
            MANIPULATOR_PIVOT.GEARING,
            Preferences.MANIPULATOR_PIVOT.MIN_ANGLE,
            Preferences.MANIPULATOR_PIVOT.MAX_ANGLE,
            Rotation2d.fromDegrees(0.25),
            true
        );
    }

    @Override
    public void periodic() {
      if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
      if (isCalibrating) return;
      if (RobotState.isDisabled()) {
        controller.setTargetAngle(controller.getPosition());
      }
  
      controller.run();
  
      // motor.set(0.33);
      
      if (motor.getAppliedOutput() > 0.0 && getPosition().getRadians() > Preferences.SHOOTER_PIVOT.MAX_ANGLE.getRadians()) {
        motor.stopMotor();
      }
  
      if (motor.getAppliedOutput() < 0.0 && getPosition().getRadians() < Preferences.SHOOTER_PIVOT.MIN_ANGLE.getRadians()) {
        motor.stopMotor();
      }
  
      if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SHOOTER) motor.stopMotor();
    }

    public Command setTargetAngleCommand(Supplier<Rotation2d> angleSupplier) {
        return Commands.runEnd(
          () -> setTargetAngle(angleSupplier.get()),
          () -> setTargetAngle(Preferences.SHOOTER_PIVOT.IDLE_ANGLE)
        );
      }
    
    public boolean isAngleAchievable(Rotation2d angle) {
        if (angle == null) return false;
        return controller.isAngleAchievable(angle);
    }

    public void setTargetAngle(Rotation2d angle) {
        if (angle == null) return;
            controller.setTargetAngle(angle);
    }

    public Rotation2d getTargetAngle() {
        return controller.getTargetAngle();
    }

    public Rotation2d getPosition() {
        return controller.getPosition();
    }

    public boolean doneMoving() {
        return controller.doneMoving();
    }

    public void setMaxAngle(Rotation2d angle) {
        controller.setMaxAngle(angle);
    }
}
