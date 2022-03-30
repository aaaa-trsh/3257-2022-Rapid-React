package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveAnglePID extends PIDCommand {
    public DriveAnglePID(double degrees, PIDController controller, Drivetrain drivetrain) {
        super(
            controller,
            drivetrain::getHeading,
            degrees,
            (output) -> drivetrain.arcadeDrive(0, Math.copySign(Math.min(Math.abs(output), 0.5), output)),
            drivetrain
        );
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}