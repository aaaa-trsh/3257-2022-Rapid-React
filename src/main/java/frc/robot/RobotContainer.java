package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intestines;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.utils.control.XboxJoystick;
import frc.robot.utils.tunables.TunableNumber;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    private final Intestines intestines  = new Intestines();
    private final Climber climber        = new Climber();
    private final Shooter shooter        = new Shooter();
    private final Intake intake          = new Intake();

    private boolean intestinesOverride = false;
    private TunableNumber shooterRatio = new TunableNumber("Shooter/Ratio", 0.);
    private TunableNumber magPercent = new TunableNumber("Shooter/MagSpeed", 0.5);
    
    private final XboxJoystick driverController = new XboxJoystick(IOConstants.driverControllerPort);
    private final XboxJoystick operatorController = new XboxJoystick(IOConstants.operatorControllerPort);
    private double shooterPwr = 1200;
    public RobotContainer() {
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.arcadeDrive(
                    -driverController.getLeftStickYValue(),
                    -driverController.getRightStickXValue() * .6
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
        /**
         * CONTROLS
         *      LBUMPER - MAGAZINE IN
         *      RBUMPER - MAGAZINE + INTAKE OUT
         *      RTRIGGER - INTAKE UP
         *      X - AUTO ALIGN SHOOTER
         *      LTRIGGER - SHOOTER ON
         *      DPAD - VERTICAL: ARM1, HORIZONTAL: ARM2
         *      Y - SHOOTER RPM UP
         *      A - SHOOTER RPM DOWN
         **/

        // LBUMPER - MAGAZINE IN
        operatorController.xButton
            .whenActive(() -> { System.out.println("aa");intestines.setMagazinePercent(0.2); intestinesOverride = true; }, intestines)
            .whenInactive(() -> { intestines.setMagazinePercent(0); intestinesOverride = false; }, intestines);

        // RBUMPER - MAGAZINE OUT
        operatorController.rightBumper
            .whenActive(() -> { intestines.setMagazinePercent(-.5); intestinesOverride = true; }, intestines)
            .whenInactive(() -> { intestines.setMagazinePercent(0); intestinesOverride = false; }, intestines);
        
        // RBUMPER - INTAKE OUT
        driverController.leftBumper
            .whenActive(() -> { intake.setRollerPercent(-.3);}, intestines)
            .whenInactive(() -> { intake.setRollerPercent(0);}, intestines);

        // RTRIGGER - INTAKE UP
        driverController.rightTriggerButton
            .whenActive(
                new RunCommand(() -> {
                    // intake.setIntakeState(driverController.getRightTriggerValue() < 0.75 ? IntakeState.HALF : IntakeState.DOWN);
                    intake.setIntakeState(IntakeState.DOWN);
                    intake.setRollerPercent(.7);
                }, intake)
            )
            .whenInactive(() -> { intake.setIntakeState(Intake.IntakeState.UP); intake.setRollerPercent(0); }, intake);
        
        // X - AUTO ALIGN SHOOTER
        // driverController.xButton
        //     .whenActive(
        //         new PIDCommand(
        //             new PIDController(0.1, 0, 0),
        //             () ->shooter.getLimelight().getPitchError(),
        //             0.,
        //             (output) -> { drivetrain.arcadeDrive(output, -driverController.getY()); },
        //             drivetrain
        //         )
        //     );
        
        // LTRIGGER - SHOOTER ON
        operatorController.leftTriggerButton
            .whileHeld(new InstantCommand(() -> {
                if (shooter.getLimelight().hasTarget()) {
                    shooter.setShooterFromDistance(shooter.getLimelight().getPitchError());
                }// else {
                    // shooter.setShooterSpeeds(shooterPwr * (1.-shooterRatio.get()), shooterPwr * shooterRatio.get());
                // }
            }, shooter))
            .whenInactive(() -> shooter.setShooterSpeeds(0, 0));

        // DPAD - VERTICAL: ARM1, HORIZONTAL: ARM2
        // driverController.yButton
        //     .whenActive(() -> climber.setArm1(1), climber)
        //     .whenInactive(() -> climber.setArm1(0), climber);
        // driverController.aButton
        //     .whenActive(() -> climber.setArm1(-1), climber)
        //     .whenInactive(() -> climber.setArm1(0), climber);
        // driverController.Dpad.Right
        //     .whenActive(() -> climber.setArm2(0.1), climber);
        // driverController.Dpad.Left
        //     .whenActive(() -> climber.setArm2(-0.1), climber);

        // Y - SHOOTER RPM UP
        operatorController.yButton
            .whenActive(new InstantCommand(() -> {shooterPwr += 100; System.out.println("SHOOTER PWR UP. NOW " + shooterPwr);}));
        // A - SHOOTER RPM DOWN
        operatorController.aButton
            .whenActive(new InstantCommand(() -> {shooterPwr -= 100; System.out.println("SHOOTER PWR DOWN. NOW " + shooterPwr);}));
    }

    public void periodic() {
        if (shooterRatio.hasChanged()) {
            System.out.println("SHOOTER RATIO - LOW: " + Math.round(shooterRatio.get() * 100)/100. + " HIGH:" + Math.round((1 - shooterRatio.get()) * 100)/100.);
        }
        if (magPercent.hasChanged()) {
            System.out.println("MAG PERCENT -  " + Math.round(magPercent.get() * 100)/100.);
        }
    }
    public Command getAutonomousCommand() {
        return null;
    }
}
