package frc.robot.util.hardware.MotionControl;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.hardware.linear.LinearRatios;

public interface LinearController extends Subsystem {
    public void setTargetHeight(Distance height);
    public Distance getTargetHeight();
    public boolean isHeightAchievable(Distance height);
    public Distance getMeasuredHeight();
    public void moveUp();
    public void moveDown();
    public boolean doneMoving();
    public void stopMotors();

    public static LinearController get(
        String name,
        int leftCAN,
        int rightCAN,
        int absoluteEncoderDIO,
        double absolutePositionOffset,
        LinearRatios ratios,
        Distance minHeight,
        Distance maxHeight,
        Distance tolerance,
        BooleanSupplier isEnabled,
        RealLinearController.ControlConstants controlConstants
    ) {
        if (RobotBase.isSimulation()) {
            return new SimLinearController(
                name,
                minHeight,
                maxHeight,
                tolerance
            );
        } else {
            return new RealLinearController(
                leftCAN, rightCAN, absoluteEncoderDIO,
                absolutePositionOffset,
                ratios,
                minHeight, maxHeight, tolerance,
                controlConstants
            );

            // return new DualLinearController(
            //     name,
            //     leftCAN,
            //     rightCAN,
            //     absoluteEncoderDIO,
            //     absolutePositionOffset,
            //     kP,
            //     kS,
            //     gearing,
            //     mechanismToSensor,
            //     minHeight,
            //     maxHeight,
            //     tolerance,
            //     isEnabled
            // );
        }
    }
}
