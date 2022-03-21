package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveAnglePID;
import frc.robot.commands.DriveLinePID;
import frc.robot.utils.control.AxisButton.ThresholdType;

// import frc.robot.commands.DriveAnglePID;
// import frc.robot.commands.DriveLinePID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intestines;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.control.AxisButton;
import frc.robot.utils.control.XboxJoystick;
import frc.robot.utils.control.XboxJoystick.XboxAxis;
import frc.robot.utils.tunables.TunableNumber;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    private final Intestines intestines  = new Intestines();
    // private final Climber climber        = new Climber();
    private final Shooter shooter        = new Shooter();
    private final Intake intake          = new Intake();

    private boolean intestinesOverride = false;
    private TunableNumber shooterRatio = new TunableNumber("Shooter/Ratio", 0.);
    private TunableNumber magPercent = new TunableNumber("Shooter/MagSpeed", 0.5);
    
    private final XboxJoystick driverController = new XboxJoystick(IOConstants.driverControllerPort);
    private final XboxJoystick operatorController = new XboxJoystick(IOConstants.operatorControllerPort);
    private double shooterPwr = 1200;
    
    public RobotContainer() {
        
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
                        driverController.getRightStickXValue() * .6
                    ),
                    drivetrain
                )
            ); 
            configureButtonBindings();
        }
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
                if (shooter.getLimelight().hasTarget()) { shooter.setShooterFromDistance(); }
            }, shooter))
            .whenInactive(() -> shooter.setShooterSpeeds(0, 0));
        
        new AxisButton(operatorController, XboxAxis.LEFT_Y, .5, ThresholdType.LESS_THAN)
            .whenActive(() -> {intake.setIntakeState(IntakeState.UP); System.out.println("gameign");})
            .whenInactive(() -> intake.setIntakeState(IntakeState.DOWN));
        // operatorController.Dpad.Up
        //     .whenActive(() -> climber.setArm1(1), climber)
        //     .whenInactive(() -> climber.setArm1(0), climber);
        // operatorController.Dpad.Down
        //     .whenActive(() -> climber.setArm1(-1), climber)
        //     .whenInactive(() -> climber.setArm1(0), climber);
        // driverController.Dpad.Right
        //     .whenActive(() -> climber.setArm2(1), climber);
        // operatorController.Dpad.Left
        //     .whenActive(() -> climber.setArm2(-1), climber);

        
        // DRIVER ------------------------------------------
        driverController.rightTriggerButton
            .whenActive(() -> { intake.setRollerPercent(.7);}, intestines)
            .whenInactive(() -> { intake.setRollerPercent(0);}, intestines);
        driverController.leftTriggerButton
            .whenActive(() -> { intake.setRollerPercent(-.3);}, intestines)
            .whenInactive(() -> { intake.setRollerPercent(0);}, intestines);
        driverController.aButton
            .whenActive(
                new PIDCommand(
                    new PIDController(0.03, 0.06, 0),
                    () ->shooter.getLimelight().getYawError(),
                    0.,
                    (output) -> { 
                        drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.5), output));
                    },
                    drivetrain
                ).withTimeout(2)
            )
            .whenInactive(() -> drivetrain.tankDrive(0, 0));        
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
        driverController.leftTriggerButton
            .whileHeld(new InstantCommand(() -> {
                if (shooter.getLimelight().hasTarget()) {
                    shooter.setShooterFromDistance();
                }// else {
                    // shooter.setShooterSpeeds(shooterPwr * (1.-shooterRatio.get()), shooterPwr * shooterRatio.get());
                // }
            }, shooter))
            .whenInactive(() -> shooter.setShooterSpeeds(0, 0));
        
        // RTRIGGER - INTAKE DOWN
        driverController.rightTriggerButton
            .whenActive(
                new RunCommand(() -> {
                    // intake.setIntakeState(driverController.getRightTriggerValue() < 0.75 ? IntakeState.HALF : IntakeState.DOWN);
                    // System.out.println(driverController.getRightTriggerValue() < 0.75 ? "Intake Lift HALFWAY" : "Intake Lift FULL DOWN");
                    intake.setIntakeState(IntakeState.DOWN);
                    intake.setRollerPercent(.7);
                }, intake)
            )
            .whenInactive(() -> { intake.setIntakeState(Intake.IntakeState.UP); intake.setRollerPercent(0); }, intake);
            
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
                // new RunCommand(() -> {
                //     double output = 0;
                //     if (shooter.getLimelight().hasTarget()) { output = pid.calculate(shooter.getLimelight().getYawError(), 0); }
                //     drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.5), output));
                // }, drivetrain).withTimeout(2)
                new PIDCommand(
                    new PIDController(0.03, 0.06, 0),
                    () ->shooter.getLimelight().getYawError(),
                    0.,
                    (output) -> { 
                        drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -Math.copySign(Math.min(Math.abs(output), 0.5), output));
                    },
                    drivetrain
                ).withTimeout(2)
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
        if (magPercent.hasChanged()) {
            System.out.println("MAG PERCENT -  " + Math.round(magPercent.get() * 100)/100.);
        }
    }
    public Command getAutonomousCommand() {
        // 1 BALL AUTO ------------------------
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> {
        //         intake.setIntakeState(IntakeState.DOWN);
        //         intake.setRollerPercent(0.7);
        //     }),
        //     new ParallelCommandGroup(
        //         new PIDCommand(drivetrain.getTurnController(), () -> shooter.getLimelight().getPitchError(), 0., (output) -> drivetrain.arcadeDrive(output, 0), drivetrain),
        //         new RunCommand(() -> {
        //             shooter.setShooterFromDistance();
        //             var flywheelError = shooter.getFlywheelNativeVelocityError();
        //             if ((flywheelError.getFirst() + flywheelError.getSecond()) < ShooterConstants.rpmTolerance) {
        //                 intestines.setMagazinePercent(0.2);
        //             } else {
        //                 intestines.setMagazinePercent(0);
        //             }
        //         }, shooter)
        //     ).withTimeout(5),
        //     new DriveLinePID(-1, 0.1, 0.7, drivetrain)
        // );

        // 2 BALL AUTO ------------------------
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                intake.setIntakeState(IntakeState.DOWN);
                intake.setRollerPercent(0.7);
            }),
            new DriveLinePID(1, 0.1, 0.7, drivetrain),
            new DriveAnglePID(180, new PIDController(.1, 0, 0), drivetrain),
            new ParallelCommandGroup(
                new PIDCommand(pid, () -> shooter.getLimelight().getPitchError(), 0., (output) -> drivetrain.arcadeDrive(0, -Math.copySign(Math.min(Math.abs(output), 0.5), output)), drivetrain),
                new RunCommand(() -> {
                    shooter.setShooterFromDistance();
                    var flywheelError = shooter.getFlywheelNativeVelocityError();
                    if ((flywheelError.getFirst() + flywheelError.getSecond()) < ShooterConstants.rpmTolerance) {
                        intestines.setMagazinePercent(0.2);
                    } else {
                        intestines.setMagazinePercent(0);
                    }
                }, shooter)
            )
        );
        // return null;
    }
}
