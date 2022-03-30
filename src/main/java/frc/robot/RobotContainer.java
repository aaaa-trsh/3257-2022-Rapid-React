package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.utils.control.AxisButton;
import frc.robot.utils.control.AxisButton.ThresholdType;
import frc.robot.utils.control.XboxJoystick;
import frc.robot.utils.control.XboxJoystick.XboxAxis;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    private final Intestines intestines  = new Intestines();
    private final Climber climber        = new Climber();
    private final Shooter shooter        = new Shooter();
    private final Intake intake          = new Intake();
    
    private final XboxJoystick driverController = new XboxJoystick(IOConstants.driverControllerPort);
    private final XboxJoystick operatorController = new XboxJoystick(IOConstants.operatorControllerPort);

    private double shooterPwr = 1200;
    private double shooterRatio = 0;
    
    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    private PIDController visionPID = new PIDController(0.03, 0.06, 0);

    public RobotContainer() {
        configureAutonomousChooser();
        if (Constants.debugControl) {
            drivetrain.setDefaultCommand(
                new RunCommand(
                    () -> drivetrain.arcadeDrive(
                        driverController.getLeftStickYValue(),
                        driverController.getLeftStickXValue() * .7
                    ),
                    drivetrain
                )
            );
            configureDebugButtonBindings();
        } else {
            drivetrain.setDefaultCommand(
                new RunCommand(
                    () -> drivetrain.arcadeDrive(
                        driverController.getLeftStickYValue(),
                        driverController.getRightStickXValue() * .7
                    ),
                    drivetrain
                )
            ); 
        }
        SmartDashboard.putData("auto choser",autoChooser);
        configureButtonBindings();
    }
    public void autoInit() {
        drivetrain.resetGyro();
    }
    private void configureAutonomousChooser() {
        Command auto1Ball = new SequentialCommandGroup(
            new RunCommand(() -> drivetrain.tankDrive(.9,.9), drivetrain).withTimeout(.8),
            new ParallelCommandGroup(
                new RunCommand(() -> {
                    double output = 0.3;
                    shooter.getLimelight().setLightState(0);
                    if (shooter.getLimelight().hasTarget()) { output = visionPID.calculate(shooter.getLimelight().getYawError(), 0); }
                    drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.5), output));
                }, drivetrain),
                new RunCommand(() -> {
                    if (shooter.getLimelight().hasTarget())
                        shooter.setShooterFromDistance();
                }, shooter),
                new WaitCommand(1).andThen(new InstantCommand(()->intestines.setMagazinePercent(0.1)))
            ).withTimeout(10),
            new InstantCommand(() -> {
                shooter.setShooterSpeeds(0, 0);
                intestines.setMagazinePercent(0);
            })
        );
        Command auto2Ball = new SequentialCommandGroup(
            new RunCommand(() -> drivetrain.tankDrive(.9,.9), drivetrain).withTimeout(.2),
            new InstantCommand(() -> drivetrain.tankDrive(0, 0), intake),
            new WaitCommand(0.6),
            new InstantCommand(() -> intake.setRollerPercent(-1), intake),
            new RunCommand(() -> drivetrain.tankDrive(-.7,-.7), drivetrain).withTimeout(1.6),
            new InstantCommand(() -> intake.setRollerPercent(0), intake),
            new ParallelCommandGroup(
                new RunCommand(() -> {
                    double output = 1;
                    shooter.getLimelight().setLightState(0);
                    if (shooter.getLimelight().hasTarget()) { output = visionPID.calculate(shooter.getLimelight().getYawError(), 0); }
                    drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.7), output));
                }, drivetrain),
                new RunCommand(() -> {
                    if (shooter.getLimelight().hasTarget())
                        shooter.setShooterFromDistance();
                }, shooter),
                new WaitCommand(2).andThen(new InstantCommand(()->intestines.setMagazinePercent(0.1)))
            ).withTimeout(10),
            new InstantCommand(() -> {
                shooter.setShooterSpeeds(0, 0);
                intestines.setMagazinePercent(0);
            })
        );
        // Command autoRude2Ball = new SequentialCommandGroup();
        Command auto3Ball = new SequentialCommandGroup(
            new RunCommand(() -> drivetrain.tankDrive(.9,.9), drivetrain).withTimeout(.2),
            new InstantCommand(() -> drivetrain.tankDrive(0, 0), intake),
            new WaitCommand(.6),
            new InstantCommand(() -> intake.setRollerPercent(-1), intake),
            new RunCommand(() -> drivetrain.tankDrive(-.7,-.7), drivetrain).withTimeout(1.6),
            new InstantCommand(() -> intake.setRollerPercent(0), intake),
            new ParallelCommandGroup(
                new RunCommand(() -> {
                    double output = 1;
                    shooter.getLimelight().setLightState(0);
                    if (shooter.getLimelight().hasTarget()) { output = visionPID.calculate(shooter.getLimelight().getYawError(), 0); }
                    drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.7), output));
                }, drivetrain),
                new RunCommand(() -> {
                    if (shooter.getLimelight().hasTarget())
                        shooter.setShooterFromDistance();
                }, shooter),
                new WaitCommand(2).andThen(new InstantCommand(()->intestines.setMagazinePercent(0.1)))
            ).withTimeout(5),
            new InstantCommand(() -> {
                shooter.setShooterSpeeds(0, 0);
                intestines.setMagazinePercent(0);
            }),
            new DriveAnglePID(drivetrain.getHeading() + 53, new PIDController(0.05, 0.05, 0), drivetrain).withTimeout(1),
            new RunCommand(() -> drivetrain.tankDrive(-.7,-.7), drivetrain).withTimeout(2),
            new ParallelCommandGroup(
                new RunCommand(() -> {
                    double output = 1;
                    shooter.getLimelight().setLightState(0);
                    if (shooter.getLimelight().hasTarget()) { output = visionPID.calculate(shooter.getLimelight().getYawError(), 0); }
                    drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.7), output));
                }, drivetrain),
                new RunCommand(() -> {
                    if (shooter.getLimelight().hasTarget())
                        shooter.setShooterFromDistance();
                }, shooter),
                new WaitCommand(2).andThen(new InstantCommand(()->intestines.setMagazinePercent(0.1)))
            ).withTimeout(3),
            new InstantCommand(() -> {
                shooter.setShooterSpeeds(0, 0);
                intestines.setMagazinePercent(0);
            })
        );
        
        autoChooser.addOption("1 Ball", auto1Ball);
        autoChooser.addOption("2 Ball", auto2Ball);
        autoChooser.addOption("3 Ball", auto3Ball);
        // autoChooser.addOption("Rude 2 Ball", auto2Ball);
    }

    private void configureButtonBindings() {
        // OPERATOR ------------------------------------------
        operatorController.rightBumper
            .whenActive(() -> intestines.setMagazinePercent(0.2), intestines)
            .whenInactive(() -> intestines.setMagazinePercent(0), intestines);

        operatorController.leftBumper
            .whenActive(() -> intestines.setMagazinePercent(-0.5), intestines)
            .whenInactive(() -> intestines.setMagazinePercent(0), intestines);
        
        operatorController.rightTriggerButton
            .whileHeld(new InstantCommand(() -> { if (shooter.getLimelight().hasTarget()) { shooter.setShooterFromDistance(); }}, shooter))
            .whenInactive(() -> shooter.setShooterSpeeds(0, 0));
            
        operatorController.selectButton
            .whileHeld(new InstantCommand(() -> shooter.setShooterFromDistance(-4), shooter))
            .whenInactive(() -> shooter.setShooterSpeeds(0, 0));

        operatorController.startButton
            .whileHeld(() -> shooter.setShooterSpeeds(
                shooterPwr * (1. - shooterRatio), shooterPwr * shooterRatio), shooter)
            .whenInactive(() -> shooter.setShooterSpeeds(0, 0));
        
        new AxisButton(operatorController, XboxAxis.LEFT_Y, -0.5, ThresholdType.LESS_THAN)
            .whenActive(() -> intake.setIntakeState(IntakeState.UP));
        new AxisButton(operatorController, XboxAxis.LEFT_Y, 0.5, ThresholdType.GREATER_THAN)
            .whenActive(() -> intake.setIntakeState(IntakeState.DOWN));

        operatorController.yButton.whenActive(() -> climber.setArm1( 1), climber).whenInactive(() -> climber.setArm1(0), climber);
        operatorController.aButton.whenActive(() -> climber.setArm1(-1), climber).whenInactive(() -> climber.setArm1(0), climber);
        operatorController.xButton.whenActive(() -> climber.setArm2( 1), climber).whenInactive(() -> climber.setArm2(0), climber);
        operatorController.bButton.whenActive(() -> climber.setArm2(-1), climber).whenInactive(() -> climber.setArm2(0), climber);
        operatorController.Dpad.Up  .whenActive(() -> {shooterPwr += 50; System.out.println("shooter speed++ | now:" + shooterPwr);});
        operatorController.Dpad.Down.whenActive(() -> {shooterPwr -= 50; System.out.println("shooter speed-- | now:" + shooterPwr);});
        operatorController.Dpad.Left .whenActive(() -> {shooterRatio += 0.05; System.out.println("shooter ratio++ | now:" + shooterRatio);});
        operatorController.Dpad.Right.whenActive(() -> {shooterRatio -= 0.05; System.out.println("shooter ratio-- | now:" + shooterRatio);});


        // DRIVER ------------------------------------------
        driverController.rightTriggerButton
            .whenActive(() -> intake.setRollerPercent(-1), intestines)
            .whenInactive(() -> intake.setRollerPercent(0), intestines);
        driverController.leftTriggerButton
            .whenActive(() -> intake.setRollerPercent(0.3), intestines)
            .whenInactive(() -> intake.setRollerPercent(0), intestines);
        driverController.aButton
            .whenActive(
                new RunCommand(() -> {
                    double output = -driverController.getRightStickXValue();
                    if (shooter.getLimelight().hasTarget()) { output = visionPID.calculate(shooter.getLimelight().getYawError(), 0); } // Math.toDegrees(Math.atan((Math.exp(0.052f)*shooter.getLimelight().getPitchError())/7))
                    drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.7), output));
                }, drivetrain).withTimeout(2)
            ).whenInactive(() -> drivetrain.tankDrive(0, 0));        
    }

    private void configureDebugButtonBindings() {
        // OPERATOR ------------------------------------------
        driverController.rightBumper
            .whenActive(() -> intestines.setMagazinePercent(0.2), intestines)
            .whenInactive(() -> intestines.setMagazinePercent(0), intestines);
        
        driverController.rightTriggerButton
            .whileHeld(new InstantCommand(() -> {
                if (shooter.getLimelight().hasTarget()) { shooter.setShooterFromDistance(); }
                else { shooter.setShooterSpeeds((shooterPwr * (1. - shooterRatio)) + 40, (shooterPwr * shooterRatio) + 40); }
            }, shooter)).whenInactive(() -> { shooter.setShooterSpeeds(0, 0); });

        driverController.Dpad.Up  .whenActive(() -> {shooterPwr += 50; System.out.println("shooter speed++ | now:" + shooterPwr);});
        driverController.Dpad.Down.whenActive(() -> {shooterPwr -= 50; System.out.println("shooter speed-- | now:" + shooterPwr);});
        driverController.Dpad.Left .whenActive(() -> {shooterRatio += 0.05; System.out.println("shooter ratio++ | now:" + shooterRatio);});
        driverController.Dpad.Right.whenActive(() -> {shooterRatio -= 0.05; System.out.println("shooter ratio-- | now:" + shooterRatio);});

        driverController.yButton.whenActive(() -> climber.setArm1( 1), climber).whenInactive(() -> climber.setArm1(0), climber);
        driverController.aButton.whenActive(() -> climber.setArm1(-1), climber).whenInactive(() -> climber.setArm1(0), climber);
        driverController.xButton.whenActive(() -> climber.setArm2( 1), climber).whenInactive(() -> climber.setArm2(0), climber);
        driverController.bButton.whenActive(() -> climber.setArm2(-1), climber).whenInactive(() -> climber.setArm2(0), climber);
        
        driverController.leftTriggerButton.whenActive(() -> intake.setRollerPercent(-1), intestines).whenInactive(() -> intake.setRollerPercent(0), intestines);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
