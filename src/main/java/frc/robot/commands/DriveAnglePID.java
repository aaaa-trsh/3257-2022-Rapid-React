package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveAnglePID extends PIDCommand {

    public DriveAnglePID(double degrees, Drivetrain drivetrain) {
        super(
            drivetrain.getTurnController(),
            drivetrain::getHeading,
            degrees,
            (output) -> drivetrain.arcadeDrive(0, output),
            drivetrain
        );
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}