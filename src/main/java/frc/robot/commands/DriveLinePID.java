package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveLinePID extends CommandBase {
    private Drivetrain drivetrain;
    private double initialLeft, initialRight;
    private double targetLeft, targetRight;
    private double distance;
    private PIDController controller;
    private double tolerance, maxOutput;

    public DriveLinePID(PIDController controller, double distance, double tolerance, double maxOutput, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.tolerance = tolerance;
        this.controller = controller;
        this.distance = distance;

        this.initialLeft = drivetrain.getLeftPosition();
        this.targetLeft = (initialLeft + distance);
        this.initialRight = drivetrain.getRightPosition();
        this.targetRight = (initialRight + distance);
    }

    public double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val)); }

    @Override
    public void execute() {
        System.out.println(
            " init left:" + this.initialLeft +
            " distance: " + this.distance + 
            " target left: " + Drivetrain.nativePositionToMeters(this.targetLeft) + 
            " out: " + clamp(controller.calculate(drivetrain.getLeftPosition(), targetLeft), -maxOutput, maxOutput) 
        );
        System.out.println("DRIVE... NOW!   " + controller.calculate(drivetrain.getLeftPosition(), targetLeft) + " | " + controller.calculate(drivetrain.getRightPosition(), targetRight));
        drivetrain.tankDrive(
            clamp(controller.calculate(drivetrain.getLeftPosition(), targetLeft), -maxOutput, maxOutput), 
            clamp(controller.calculate(drivetrain.getRightPosition(), targetRight), -maxOutput, maxOutput)
        );
    }

    @Override
    public boolean isFinished() {
        return false;//Math.abs((((drivetrain.getLeftPosition() - initialLeft) + (drivetrain.getRightPosition() - initialRight)) / 2.) - Drivetrain.metersToNativePosition(distance)) < tolerance;
    }
}
