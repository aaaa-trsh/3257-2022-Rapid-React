package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.DriveAnglePID;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intestines;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.utils.control.XboxJoystick;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    private final Intestines intestines  = new Intestines();
    private final Climber climber        = new Climber();
    private final Shooter shooter        = new Shooter();
    private final Intake intake          = new Intake();
    
    private final XboxJoystick driverController = new XboxJoystick(IOConstants.driverControllerPort);
    private final XboxJoystick operatorController = new XboxJoystick(IOConstants.operatorControllerPort);
    
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private static enum ControlMode {
		CO_OP, SOLO;
	}
    private ControlMode controlMode = ControlMode.SOLO;

    public RobotContainer() {
        configureAutonomousChooser();

        // drive command by default;
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.arcadeDrive(
                    driverController.getLeftStickYValue(),
                    driverController.getRightStickXValue() * .7
                ),
                drivetrain
            )
        );
        
        // intake mgmt by default
        intestines.setDefaultCommand(
            new RunCommand(() -> {
                // manual overrides
                if (
                    (controlMode == ControlMode.SOLO && driverController.leftBumper.get()) ||
                    operatorController.rightBumper.get())
                { // intake
                    intestines.setMagazinePercent(0.2);
                } else if (
                    (controlMode == ControlMode.CO_OP ? driverController.leftTriggerButton.get() : driverController.selectButton.get()) || // TODO: remove driver control when intake lift works!
                    operatorController.leftTriggerButton.get()) 
                {// eject
                    intestines.setMagazinePercent(-0.3);
                } else {
                    // auto mag
                    if (intestines.isBallInQueue()) {
                        intestines.setMagazinePercent(0.2);
                    } else { // reset by default
                        intestines.setMagazinePercent(0);
                    }
                }
            }, intestines)
        );

        if (controlMode == ControlMode.SOLO) {
            configureSoloButtonBindings();
        } else if (controlMode == ControlMode.CO_OP) {
            configureCoOpButtonBindings();
        }

        SmartDashboard.putData("auto choser", autoChooser);
    }
    public void autoInit() {
        drivetrain.resetGyro();
    }
    private void configureAutonomousChooser() {
        Command auto1Ball = new SequentialCommandGroup(
            shooter.spinUpWithVisionCommand(),
            new RunCommand(() -> drivetrain.tankDrive(.9,.9), drivetrain).withTimeout(.8),
            drivetrain.aimWithVisionCommand(shooter.getLimelight(), () -> 0.3, () -> 0).withTimeout(1),
            new InstantCommand(()->intestines.setMagazinePercent(0.1)),
            new WaitCommand(10),
            new InstantCommand(() -> {
                shooter.setShooterSpeeds(0, 0);
                intestines.setMagazinePercent(0);
            })
        );
        Command auto2Ball = new SequentialCommandGroup(
            // drop intake
            new RunCommand(() -> drivetrain.tankDrive(.9,.9), drivetrain).withTimeout(.2),
            new InstantCommand(() -> drivetrain.tankDrive(0, 0), drivetrain),
            // TODO: replace with intake down command w/ motor
            new WaitCommand(0.3),
            shooter.spinUpWithVisionCommand(),
            new WaitCommand(0.3),
            new InstantCommand(() -> intake.setRollerPercent(1), intake),
            new RunCommand(() -> drivetrain.tankDrive(-.7,-.7), drivetrain).withTimeout(1.6),
            new InstantCommand(() -> intake.setRollerPercent(0), intake),
            drivetrain.aimWithVisionCommand(shooter.getLimelight(), () -> 1, () -> 0).withTimeout(2), // TODO: replace aimWithVision timeout with isFinished flag
            new InstantCommand(()->intestines.setMagazinePercent(0.1)),
            new WaitCommand(4),
            new InstantCommand(() -> {
                shooter.setShooterSpeeds(0, 0);
                intestines.setMagazinePercent(0);
            })
        );

        Command auto3Ball = new SequentialCommandGroup(
            auto2Ball,
            shooter.spinUpWithVisionCommand(),
            new DriveAnglePID(drivetrain.getHeading() + 53, drivetrain.getTurnController(), drivetrain).withTimeout(1),
            new InstantCommand(() -> intake.setRollerPercent(1), intake),
            new RunCommand(() -> drivetrain.tankDrive(-.7,-.7), drivetrain).withTimeout(2),
            new InstantCommand(() -> intake.setRollerPercent(0), intake),
            new RunCommand(() -> drivetrain.tankDrive(1,1), drivetrain).withTimeout(0.3),
            drivetrain.aimWithVisionCommand(shooter.getLimelight(), () -> 1, () -> 0).withTimeout(2),
            new InstantCommand(() -> intestines.setMagazinePercent(0.1)),
            new WaitCommand(4),
            new InstantCommand(() -> {
                shooter.setShooterSpeeds(0, 0);
                intestines.setMagazinePercent(0);
            })
        );
        autoChooser.addOption("1 Ball", auto1Ball);
        autoChooser.addOption("2 Ball", auto2Ball);
        autoChooser.addOption("3 Ball", auto3Ball);
    }
    
    public void reset() {
        shooter.setLightState(false);
        shooter.setShooterSpeeds(0, 0);
        intestines.setMagazinePercent(0);
        drivetrain.tankDrive(0, 0);
        intake.setRollerPercent(0);
    }

    private void configureCoOpButtonBindings() {
        // OPERATOR ------------------------------------------
        // operator eject command is set by default (left trigger)
        operatorController.rightTriggerButton // main shoot
            .whileHeld(shooter.spinUpWithVisionCommand())
            .whenInactive(shooter.spinDownCommand());
            
        operatorController.selectButton // manual override
            .whileHeld(shooter.spinUpOverrideCommand())
            .whenInactive(shooter.spinDownCommand());

        // climber shit
        operatorController.yButton.whenActive(() -> climber.setArm1( 1), climber).whenInactive(() -> climber.setArm1(0), climber);
        operatorController.aButton.whenActive(() -> climber.setArm1(-1), climber).whenInactive(() -> climber.setArm1(0), climber);
        operatorController.xButton.whenActive(() -> climber.setArm2( 1), climber).whenInactive(() -> climber.setArm2(0), climber);
        operatorController.bButton.whenActive(() -> climber.setArm2(-1), climber).whenInactive(() -> climber.setArm2(0), climber);

        // DRIVER ------------------------------------------
        // driver drive command is set by default (right/left joysticks)
        // driver mag command is set by default (left bumper)
        // driver eject command is set by default (start)
        driverController.rightTriggerButton
            .whenActive(() -> { // intake down + start rollers
                intake.setRollerPercent(.7);
                intake.setIntakeState(IntakeState.DOWN);
            }, intestines)
            .whenInactive(() -> { // intake up + stop rollers
                intake.setRollerPercent(0);
                intake.setIntakeState(IntakeState.UP);
            }, intestines);

        driverController.aButton
            .whenActive(
                drivetrain.aimWithVisionCommand(
                    shooter.getLimelight(),
                    driverController::getLeftStickYValue,
                    () -> { return driverController.getRightStickXValue() * .7; }
                ))
            .whenInactive(() -> { 
                drivetrain.tankDrive(0, 0);
            });
    }

    public void configureSoloButtonBindings() {
        // DRIVER ------------------------------------------
        // driver drive command is set by default (right/left joysticks)
        // driver eject command is set by default (left bumper)
        // operator eject command is set by default (left trigger)

        driverController.rightTriggerButton // main shoot
            .whileHeld(shooter.spinUpWithVisionCommand())
            .whenInactive(shooter.spinDownCommand());

        driverController.rightBumper // manual override
            .whileHeld(shooter.spinUpOverrideCommand())
            .whenInactive(shooter.spinDownCommand());

        driverController.leftTriggerButton
            .whenActive(() -> { // intake down + start rollers
                // intake.setRollerPercent(.7);
                intake.setIntakeState(IntakeState.DOWN);
            }, intestines)
            .whenInactive(() -> { // intake up + stop rollers
                intake.setRollerPercent(0);
                intake.setIntakeState(IntakeState.UP);
            }, intestines);
        
        driverController.leftBumper
            .whenActive(() -> intestines.setMagazinePercent(.5))
            .whenInactive(() -> intestines.setMagazinePercent(0));

        driverController.aButton
            .whenHeld(
                drivetrain.aimWithVisionCommand(
                    shooter.getLimelight(),
                    driverController::getLeftStickYValue,
                    () -> { return driverController.getRightStickXValue() * .7; }
                ))
            .whenInactive(() -> { 
                drivetrain.tankDrive(0, 0);
            });
        
        // climb shit
        driverController.yButton.whenActive(() -> climber.setArm1( 1), climber).whenInactive(() -> climber.setArm1(0), climber);
        driverController.aButton.whenActive(() -> climber.setArm1(-1), climber).whenInactive(() -> climber.setArm1(0), climber);
        driverController.xButton.whenActive(() -> climber.setArm2( 1), climber).whenInactive(() -> climber.setArm2(0), climber);
        driverController.bButton.whenActive(() -> climber.setArm2(-1), climber).whenInactive(() -> climber.setArm2(0), climber);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
