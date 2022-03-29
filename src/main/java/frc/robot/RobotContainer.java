package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Climber;
// import frc.robot.commands.DriveAnglePID;
// import frc.robot.commands.DriveLinePID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intestines;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.utils.control.AxisButton;
import frc.robot.utils.control.AxisButton.ThresholdType;
import frc.robot.utils.control.XboxJoystick;
import frc.robot.utils.control.XboxJoystick.XboxAxis;
import frc.robot.utils.tunables.TunableNumber;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    private final Intestines intestines  = new Intestines();
    private final Climber climber        = new Climber();
    private final Shooter shooter        = new Shooter();
    private final Intake intake          = new Intake();

    private boolean intestinesOverride = false;
    private TunableNumber shooterRatio = new TunableNumber("Shooter/Ratio", 0.);
    
    private final XboxJoystick driverController = new XboxJoystick(IOConstants.driverControllerPort);
    private final XboxJoystick operatorController = new XboxJoystick(IOConstants.operatorControllerPort);
    private double shooterPwr = 1200;
    
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureAutonomousChooser();
        if (Constants.debugControl) {
            drivetrain.setDefaultCommand(
                new RunCommand(
                    () -> drivetrain.arcadeDrive(
                        driverController.getLeftStickYValue(),
                        driverController.getLeftStickXValue() * .6
                    ),   
                    drivetrain
                )
            ); 
            configureDebugButtonBindings();
        }
        else {
            drivetrain.setDefaultCommand(
                new RunCommand(
                    () -> drivetrain.arcadeDrive(
                        driverController.getLeftStickYValue(),
                        driverController.getRightStickXValue() * .7
                    ),
                    drivetrain
                )
            ); 
            configureButtonBindings();
        }
    }

    private void configureAutonomousChooser() {
        Command auto1Ball = new SequentialCommandGroup(
            new RunCommand(() -> drivetrain.tankDrive(.9,.9), drivetrain).withTimeout(.8),
            new ParallelCommandGroup(
                new RunCommand(() -> {
                    double output = 0.3;
                    shooter.getLimelight().setLightState(0);
                    if (shooter.getLimelight().hasTarget()) { output = pid.calculate(shooter.getLimelight().getYawError(), 0); }
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
            new RunCommand(() -> drivetrain.tankDrive(.9,.9), drivetrain).withTimeout(.3)
        );
        Command autoRude2Ball = new SequentialCommandGroup(

        );
        Command auto3Ball = new SequentialCommandGroup(

        );
        
        autoChooser.addOption("1 Ball", auto1Ball);
        autoChooser.addOption("2 Ball", auto2Ball);
        autoChooser.addOption("Rude 2 Ball", auto2Ball);
        autoChooser.addOption("3 Ball", auto2Ball);
    }

    private void configureButtonBindings() {
        // OPERATOR ------------------------------------------
        operatorController.rightBumper
            .whenActive(() -> { intestines.setMagazinePercent(0.2); intestinesOverride = true; }, intestines)
            .whenInactive(() -> { intestines.setMagazinePercent(0); intestinesOverride = false; }, intestines);

        operatorController.leftBumper
            .whenActive(() -> { intestines.setMagazinePercent(-.5); intestinesOverride = true; }, intestines)
            .whenInactive(() -> { intestines.setMagazinePercent(0); intestinesOverride = false; }, intestines);
        
        operatorController.rightTriggerButton
            .whileHeld(new InstantCommand(() -> {
                // System.out.println("shooting");
                // shooter.getLimelight().setLightState(0);
                if (shooter.getLimelight().hasTarget()) { shooter.setShooterFromDistance(); }
            }, shooter))
            .whenInactive(() -> { shooter.setShooterSpeeds(0, 0);  });
            
        operatorController.selectButton
            .whileHeld(new InstantCommand(() -> {
                shooter.setShooterFromDistance(-4);
            }, shooter))
            .whenInactive(() -> { shooter.setShooterSpeeds(0, 0);  });
        
        new AxisButton(operatorController, XboxAxis.LEFT_Y, -.5, ThresholdType.LESS_THAN)
            .whenActive(() -> {intake.setIntakeState(IntakeState.UP); System.out.println("intk up");});
            // .whenInactive(() -> intake.setIntakePercent(0));
        new AxisButton(operatorController, XboxAxis.LEFT_Y, .5, ThresholdType.GREATER_THAN)
            .whenActive(() -> {intake.setIntakeState(IntakeState.DOWN); System.out.println("intk down");});
            // .whenInactive(() -> intake.setIntakePercent(0));

        operatorController.yButton
            .whenActive(() -> climber.setArm1(1), climber)
            .whenInactive(() -> climber.setArm1(0), climber);
        operatorController.aButton
            .whenActive(() -> climber.setArm1(-1), climber)
            .whenInactive(() -> climber.setArm1(0), climber);
        operatorController.xButton
            .whenActive(() -> climber.setArm2(1), climber)
            .whenInactive(() -> climber.setArm2(0), climber);
        operatorController.bButton
            .whenActive(() -> climber.setArm2(-1), climber)
            .whenInactive(() -> climber.setArm2(0), climber);

        
        // DRIVER ------------------------------------------
        driverController.rightTriggerButton
            .whenActive(() -> { intake.setRollerPercent(1);}, intestines)
            .whenInactive(() -> { intake.setRollerPercent(0);}, intestines);
        driverController.leftTriggerButton
            .whenActive(() -> { intake.setRollerPercent(-.3);}, intestines)
            .whenInactive(() -> { intake.setRollerPercent(0);}, intestines);
        driverController.aButton
        .whenActive(
            new RunCommand(() -> {
                double output = -driverController.getRightStickXValue();
                // shooter.getLimelight().setLightState(0);
                if (shooter.getLimelight().hasTarget()) { output = pid.calculate(shooter.getLimelight().getYawError(), 0); }
                drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.5), output));
            }, drivetrain).withTimeout(2)
        )
        .whenInactive(() -> {drivetrain.tankDrive(0, 0);});        
    }

    PIDController pid = new PIDController(0.03, 0.06, 0);
    public void configureDebugButtonBindings() {
        /**
         * this is for my debug needs so i dont need to manhandle two controllers at the same time -k ;)
         * 
         * CONTROLS
         *     LTRIGGER - SHOOTER ON
         *     RTRIGGER - INTAKE DOWN
         *     LBUMPER - MAGAZINE IN
         *     X - MAGAZINE OUT
         *     Y - SHOOTER RPM UP
         *     A - SHOOTER RPM DOWN
         *     B - VISION ALIGN
         */

        // LTRIGGER - SHOOTER ON
        driverController.leftBumper
            .whileHeld(new InstantCommand(() -> {
                if (shooter.getLimelight().hasTarget()) {
                    shooter.setShooterFromDistance();
                } else {
                    shooter.setShooterFromDistance(4);
                }
            })).whenInactive(()->shooter.setShooterSpeeds(0, 0));
        driverController.leftTriggerButton
            .whileHeld(new InstantCommand(() -> {
                if (shooter.getLimelight().hasTarget()) {
                    shooter.setShooterFromDistance();
                } else {
                    shooter.setShooterSpeeds(shooterPwr * (1.-shooterRatio.get()), shooterPwr * shooterRatio.get());
                }
            }, shooter))
            .whenInactive(() -> shooter.setShooterSpeeds(0, 0));
        
        // RTRIGGER - INTAKE DOWN
        driverController.rightTriggerButton
            .whenActive(
                new RunCommand(() -> {
                    // intake.setIntakeState(driverController.getRightTriggerValue() < 0.75 ? IntakeState.HALF : IntakeState.DOWN);
                    // System.out.println(driverController.getRightTriggerValue() < 0.75 ? "Intake Lift HALFWAY" : "Intake Lift FULL DOWN");
                    // intake.setIntakeState(IntakeState.DOWN);
                    intake.setRollerPercent(.7);
                }, intake)
            )
            .whenInactive(  
                new InstantCommand(() -> intake.setRollerPercent(0), intake)
                .andThen(new WaitCommand(0.5)));
           
        // driverController.yButton
        //     .whenActive(() -> climber.setArm1(1), climber)
        //     .whenInactive(() -> climber.setArm1(0), climber);
        // driverController.aButton
        //     .whenActive(() -> climber.setArm1(-1), climber)
        //     .whenInactive(() -> climber.setArm1(0), climber);
        // driverController.xButton
        //     .whenActive(() -> climber.setArm2(1), climber)
        //     .whenInactive(() -> climber.setArm2(0), climber);
        // driverController.bButton
        //     .whenActive(() -> climber.setArm2(-1), climber)
        //     .whenInactive(() -> climber.setArm2(0), climber);
        // X - MAGAZINE OUT
        driverController.xButton
            .whenActive(() -> { intestines.setMagazinePercent(-.5); intestinesOverride = true; }, intestines)
            .whenInactive(() -> { intestines.setMagazinePercent(0); intestinesOverride = false; }, intestines);
        // Y - SHOOTER RPM UP
        driverController.yButton
            .whenActive(new InstantCommand(() -> {shooterPwr += 100; System.out.println("SHOOTER PWR UP. NOW " + shooterPwr);}));
        // A - SHOOTER RPM DOWN
        driverController.aButton
            .whenActive(new InstantCommand(() -> {shooterPwr -= 100; System.out.println("SHOOTER PWR DOWN. NOW " + shooterPwr);}));
        // B - VISION ALIGN
        driverController.bButton
            .whenActive(
                new RunCommand(() -> {
                    double output = -driverController.getLeftStickXValue();
                    if (shooter.getLimelight().hasTarget()) { output = pid.calculate(shooter.getLimelight().getYawError(), 0); }
                    drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.5), output));
                }, drivetrain).withTimeout(2)
            )
            .whenInactive(() -> drivetrain.tankDrive(0, 0));

        // LBUMPER - MAGAZINE IN
        driverController.rightBumper
            .whenActive(() -> { intestines.setMagazinePercent(0.2); intestinesOverride = true; }, intestines)
            .whenInactive(() -> { intestines.setMagazinePercent(0); intestinesOverride = false; }, intestines);

    }

    public void periodic() {
        if (shooterRatio.hasChanged()) {
            System.out.println("SHOOTER RATIO - LOW: " + Math.round(shooterRatio.get() * 100)/100. + " HIGH:" + Math.round((1 - shooterRatio.get()) * 100)/100.);
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
