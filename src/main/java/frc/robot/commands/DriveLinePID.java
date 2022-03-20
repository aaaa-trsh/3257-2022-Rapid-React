package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveLinePID extends CommandBase {
    private Drivetrain drivetrain;
    private double initialLeft, initialRight;
    private double targetLeft, targetRight;
    private double distance;
    private PIDController controller;
    private double tolerance, maxOutput;

    public DriveLinePID(double distance, double tolerance, double maxOutput, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.tolerance = tolerance;
        this.distance = distance;

        this.initialLeft = drivetrain.getLeftPosition();
        this.targetLeft = initialLeft + Drivetrain.metersToNativePosition(distance);
        this.initialRight = drivetrain.getRightPosition();
        this.targetRight = initialRight + Drivetrain.metersToNativePosition(distance);
    }

    public double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val)); }

    @Override
    public void execute() {
        drivetrain.tankDrive(
            clamp(controller.calculate(drivetrain.getLeftPosition(), targetLeft), -maxOutput, maxOutput), 
            clamp(controller.calculate(drivetrain.getRightPosition(), targetRight), -maxOutput, maxOutput)
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs((((drivetrain.getLeftPosition() - initialLeft) + (drivetrain.getRightPosition() - initialRight)) / 2.) - Drivetrain.metersToNativePosition(distance)) < tolerance;
    }
}
