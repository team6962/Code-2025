package frc.robot.util.hardware.MotionControl;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.MeasureMath;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.hardware.linear.LinearRatios;

public class RealLinearController implements LinearController {
    private LinearRatios ratios;
    private Motor left;
    private Motor right;
    private ControlConstants controlConstants;
    private Distance targetHeight;
    private Distance minHeight, maxHeight, tolerance;

    public RealLinearController(
        int leftCAN,
        int rightCAN,
        int absoluteEncoderDIO,
        double absolutePositionOffset,
        LinearRatios ratios,
        Distance minHeight,
        Distance maxHeight,
        Distance tolerance,
        ControlConstants controlConstants
    ) {
        this.ratios = ratios;
        left = new Motor(leftCAN, true);
        right = new Motor(rightCAN, false);
        this.controlConstants = controlConstants;
        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
        this.tolerance = tolerance;

        Logger.logNumber(this.getName() + "/targetHeight", () -> getTargetHeight().in(Meters));
        Logger.logNumber(this.getName() + "/measuredHeight", () -> getMeasuredHeight().in(Meters));
    }

    public record ControlConstants(double kP, double kI, double kD) {
    }

    private class Motor {
        private SparkMax sparkMax;
        private RelativeEncoder encoder;
        private SparkClosedLoopController controller;

        public Motor(int canId, boolean inverted) {
            sparkMax = new SparkMax(canId, MotorType.kBrushless);

            SparkMaxConfig sparkConfig = new SparkMaxConfig();

            SparkMaxUtil.configure(sparkConfig, inverted, IdleMode.kBrake);
            SparkMaxUtil.configureEncoder(sparkConfig, ratios.rotorRotationsToCarriageMeters());
            
            sparkConfig.closedLoop.pid(controlConstants.kP, controlConstants.kI, controlConstants.kD);

            encoder = sparkMax.getEncoder();

            controller = sparkMax.getClosedLoopController();
        }

        public void moveTowardsPosition(Distance position) {
            controller.setReference(position.in(Meters), ControlType.kPosition);
        }

        public void moveAtSpeed(LinearVelocity velocity) {
            controller.setReference(velocity.in(MetersPerSecond), ControlType.kVelocity);
        }

        public void disableMovement() {
            sparkMax.disable();
        }

        public Distance getPosition() {
            return Meters.of(encoder.getPosition());
        }
    }

    @Override
    public void setTargetHeight(Distance height) {
        height = MeasureMath.clamp(height, minHeight, maxHeight);

        targetHeight = height;

        left.moveTowardsPosition(height);
        right.moveTowardsPosition(height);
    }

    @Override
    public Distance getTargetHeight() {
        return targetHeight;
    }

    @Override
    public boolean isHeightAchievable(Distance height) {
        return height.lte(maxHeight) && height.gte(minHeight);
    }

    @Override
    public Distance getMeasuredHeight() {
        return left.getPosition().plus(right.getPosition()).div(2);
    }

    @Override
    public void moveUp() {
        left.moveAtSpeed(MetersPerSecond.of(0.2));
        right.moveAtSpeed(MetersPerSecond.of(0.2));
    }

    @Override
    public void moveDown() {
        left.moveAtSpeed(MetersPerSecond.of(-0.2));
        right.moveAtSpeed(MetersPerSecond.of(-0.2));
    }

    @Override
    public boolean doneMoving() {
        return getMeasuredHeight().minus(getTargetHeight()).abs(Meters) <= tolerance.in(Meters);
    }

    @Override
    public void stopMotors() {
        left.disableMovement();
        right.disableMovement();
    }
}
