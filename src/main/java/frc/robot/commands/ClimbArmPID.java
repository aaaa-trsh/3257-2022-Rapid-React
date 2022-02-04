package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArm;
import frc.robot.subsystems.ClimbArm.ArmPose;

public class ClimbArmPID extends CommandBase {
    private ClimbArm climber;
    private double targetShoulderPos;
    private double targetElbowPos;
    
    public ClimbArmPID(double targetShoulderPos, double targetElbowPos, ClimbArm climber) {
        addRequirements(climber);
        this.climber = climber;
        this.targetElbowPos = targetElbowPos;
        this.targetShoulderPos = targetShoulderPos;
    }
    
    public ClimbArmPID(ArmPose pose, ClimbArm climber) {
        addRequirements(climber);
        this.climber = climber;
        this.targetElbowPos = pose.elbowPos;
        this.targetShoulderPos = pose.shoulderPos;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        climber.setElbowPercentOutput(climber.getElbowController().calculate(climber.getElbowEncoder().getDistance(), targetElbowPos));
        climber.setShoulderPercentOutput(climber.getShoulderController().calculate(climber.getShoulderEncoder().getDistance(), targetShoulderPos));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return climber.getElbowController().atSetpoint() && climber.getShoulderController().atSetpoint();
    }
}
