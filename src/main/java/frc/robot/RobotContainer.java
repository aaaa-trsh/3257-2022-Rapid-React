package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intestines;
import frc.robot.utils.control.XboxJoystick;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    // private final Drivetrain drivetrain  = new Drivetrain();
    private final Intestines intestines  = new Intestines();
    private final Shooter shooter        = new Shooter();
    private final XboxJoystick driverController = new XboxJoystick(IOConstants.driverControllerPort);

    public RobotContainer() {
        /* drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.arcadeDrive(
                    driverController.getY(),
                    -driverController.getX()
                ), 
                drivetrain
            )
        ); 
        // mmmmmmmm autofeed nicee
        intestines.setDefaultCommand(
            new RunCommand(
                () ->  intestines.setMagazinePercent(intestines.isBallInQueue() ? 0.3 : 0), 
                intestines
            )
        ); */
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        driverController.leftTriggerButton
            .whenActive(new InstantCommand(() -> intestines.setMagazinePercent(1), intestines))
            .whenInactive(new InstantCommand(() -> intestines.setMagazinePercent(0), intestines));

        driverController.rightBumper
            .whenActive(new InstantCommand(() -> shooter.setShooterSpeeds(2000, 2000)), shooter);
            .whenInactive(new InstantCommand(() -> shooter.setShooterSpeeds(0, 0)), shooter);
        /* driverController.leftBumper
            .whenActive(new InstantCommand(() -> intestines.actuateIntake(!intestines.getIntakeState()), intestines)); */
    }

    public Command getAutonomousCommand() {
        return new RunCommand(()->{
            drivetrain.setWheelSpeeds(new DifferentialDriveWheelSpeeds(100, 100));
            System.out.println(drivetrain.getWheelSpeeds());
        }, drivetrain);
    }
}
