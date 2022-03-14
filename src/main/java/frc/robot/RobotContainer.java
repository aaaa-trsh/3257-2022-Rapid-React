package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intestines;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.control.XboxJoystick;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    private final Intestines intestines  = new Intestines();
    private final Climber climber        = new Climber();
    private final Shooter shooter        = new Shooter();
    private final Intake intake          = new Intake();

    private boolean intestinesOverride = false;
    
    private final XboxJoystick driverController = new XboxJoystick(IOConstants.driverControllerPort);
    private double shooterPwr = 300;
    public RobotContainer() {
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.arcadeDrive(
                    -driverController.getX()*.7,
                    -driverController.getY()
                ),   
                drivetrain
            )
        ); 
        // mmmmmmmm autofeed nicee
        // intestines.setDefaultCommand(
        //     new RunCommand(
        //         () ->  {
        //             if (!intestinesOverride)
        //                 intestines.setMagazinePercent(intestines.isBallInQueue() ? 0.3 : 0);
        //             // intestines.setMagazinePercent(0.1);
        //             // intestines.setIntakePercent(-0.6);
        //         }, 
        //         intestines
        //     )
        // ); 
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // X BUTTON - MAGAZINE IN
        driverController.xButton
            .whenActive(new InstantCommand(() -> { intestines.setMagazinePercent(.5); intestinesOverride = true; }, intestines))
            .whenInactive(new InstantCommand(() -> { intestines.setMagazinePercent(0); intestinesOverride = false; }, intestines));
        
        // LEFT TRIGGER - MAGAZINE + INTAKE OUT
        driverController.leftTriggerButton
            .whenActive(new InstantCommand(() -> { intestines.setMagazinePercent(-.5); intake.setRollerPercent(-.3); intestinesOverride = true; }, intestines))
            .whenInactive(new InstantCommand(() -> { intestines.setMagazinePercent(0); intake.setRollerPercent(0); intestinesOverride = false; }, intestines));

        // RIGHT BUMPER - SHOOTER ENABLE/DISABLE
        driverController.rightBumper
            .whenActive(new InstantCommand(() -> shooter.setShooterSpeeds(shooterPwr, shooterPwr), shooter))
            .whenInactive(new InstantCommand(() -> shooter.setShooterSpeeds(0, 0), shooter));
        
        // RIGHT TRIGGER - INTAKE IN
        driverController.rightTriggerButton
            .whenActive(new InstantCommand(() -> { intake.setRollerPercent(.7); }, intake))
            .whenInactive(new InstantCommand(() -> { intake.setRollerPercent(0); }, intake));
        
        // DPAD UP - INTAKE UP
        driverController.Dpad.Up
            .whenActive(new InstantCommand(() -> intake.setIntakeUp(true), intake));
        // DPAD DOWN - INTAKE DOWN
            driverController.Dpad.Down
            .whenActive(new InstantCommand(() -> intake.setIntakeUp(false), intake));

        // driverController.Dpad.Up
        //     .whenActive(new InstantCommand(() -> climber.setArm1(0.1), climber));
        // driverController.Dpad.Down
        //     .whenActive(new InstantCommand(() -> climber.setArm1(-0.1), climber));
        // driverController.Dpad.Left
        //     .whenActive(new InstantCommand(() -> climber.setArm2(0.1), climber));
        // driverController.Dpad.Right
        //     .whenActive(new InstantCommand(() -> climber.setArm2(-0.1), climber));

        // Y BUTTON - SHOOTER RPM UP
        driverController.yButton
            .whenActive(new InstantCommand(() -> {shooterPwr += 200; System.out.println("SHOOTER PWR UP. NOW " + shooterPwr);}));
        // A BUTTON - SHOOTER RPM DOWN
        driverController.aButton
            .whenActive(new InstantCommand(() -> {shooterPwr -= 200; System.out.println("SHOOTER PWR DOWN. NOW " + shooterPwr);}));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
