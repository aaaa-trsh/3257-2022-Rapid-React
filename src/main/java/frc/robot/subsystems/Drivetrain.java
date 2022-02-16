package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.Conversions.TalonFXConversions;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX backLeft = new WPI_TalonFX(DriveConstants.backLeftPort);
    private WPI_TalonFX backRight = new WPI_TalonFX(DriveConstants.backRightPort);
    private WPI_TalonFX frontLeft = new WPI_TalonFX(DriveConstants.frontLeftPort);
    private WPI_TalonFX frontRight = new WPI_TalonFX(DriveConstants.frontRightPort);

    private DifferentialDrive differentialDrive = new DifferentialDrive((MotorController)frontLeft, (MotorController)frontRight);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackwidth);
    
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
    // public static DifferentialDrivePoseEstimator odometry = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
    //     new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
    //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
    //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.sVolts, DriveConstants.vVoltSecondsPerMeter, DriveConstants.aVoltSecondsSquaredPerMeter);

    private PIDController leftController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
    private PIDController rightController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);

    private AHRS gyro = new AHRS();

    private Field2d field = new Field2d();

    public Drivetrain() {
        // Calibrate n reset the gyro
        gyro.calibrate();
        gyro.reset();
        
        // Reset all the drivetrain controllers
        backLeft.configFactoryDefault();
        frontLeft.configFactoryDefault();
        backRight.configFactoryDefault();
        frontRight.configFactoryDefault();

        frontLeft.setInverted(InvertType.None);
        frontLeft.setNeutralMode(NeutralMode.Coast);
        frontLeft.configOpenloopRamp(1/2);
        frontLeft.setSensorPhase(true);

        frontLeft.config_kP(0, DriveConstants.talonFXP, 30);
        frontLeft.config_kI(0, DriveConstants.talonFXI, 30);
        frontLeft.config_kD(0, DriveConstants.talonFXD, 30);
        frontLeft.config_kF(0, DriveConstants.talonFXF, 30);

        backLeft.follow(frontLeft);
        backLeft.setNeutralMode(NeutralMode.Coast);
        backLeft.setInverted(InvertType.FollowMaster);
        
        frontRight.setInverted(InvertType.None);
        frontRight.setNeutralMode(NeutralMode.Coast);
        frontRight.configOpenloopRamp(1/2);

        frontRight.config_kP(0, DriveConstants.talonFXP, 30);
        frontRight.config_kI(0, DriveConstants.talonFXI, 30);
        frontRight.config_kD(0, DriveConstants.talonFXD, 30);
        frontRight.config_kF(0, DriveConstants.talonFXF, 30);

        backRight.follow(frontRight);
        backRight.setInverted(InvertType.FollowMaster);
        backRight.setNeutralMode(NeutralMode.Coast);
    
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
        // odometry.update(Rotation2d.fromDegrees(getHeading()), new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity()), getLeftEncoderPosition(), getRightEncoderPosition());
        field.setRobotPose(getPose());

        SmartDashboard.putData("field", field);
        SmartDashboard.putNumber("right pos", getRightEncoderPosition());
        SmartDashboard.putNumber("left pos", getLeftEncoderPosition());
        SmartDashboard.putNumber("gyro", getHeading());
    }

    public void tankDriveVolts(double left, double right) { tankDrive(left / 12, right / 12);  }
    public void tankDrive(double left, double right) { differentialDrive.tankDrive(left, right); }
    
    public void setWheelRPM(double leftRPM, double rightRPM) {
        frontLeft.set(TalonFXControlMode.Velocity, TalonFXConversions.RPM2Native(leftRPM));
        frontRight.set(TalonFXControlMode.Velocity, TalonFXConversions.RPM2Native(rightRPM));
        differentialDrive.feed();
    }

    public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
        frontLeft.set(TalonFXControlMode.Velocity, metersPerSecondToNative(speeds.leftMetersPerSecond));
        frontRight.set(TalonFXControlMode.Velocity, metersPerSecondToNative(speeds.rightMetersPerSecond));
    }

    public void arcadeDrive(double throttle, double turn) {
        System.out.println("throttle " + Math.round(throttle*10)/10. + " turn " + Math.round(turn*10)/10. + " pose " + getPose());
        differentialDrive.arcadeDrive(throttle, turn);
    }

    public void setNeutralMode(NeutralMode mode) {
        frontLeft.setNeutralMode(mode);
        frontRight.setNeutralMode(mode);
        backLeft.setNeutralMode(mode);
        backRight.setNeutralMode(mode);
    }

    public void setSlowMode(boolean on) { differentialDrive.setMaxOutput(on ? 0.2 : 1); }
    
    public void resetOdometry(Pose2d pose) { 
        odometry.resetPosition(pose, pose.getRotation()); 
        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
        gyro.reset();
    }
    public void resetOdometry() { this.resetOdometry(new Pose2d()); }


    public Command PathCommand(Trajectory trajectory, boolean resetAtStart) {
        if (resetAtStart)
            resetOdometry(trajectory.getInitialPose());

        return new RamseteCommand(
            trajectory,
            this::getPose,
            new RamseteController(DriveConstants.ramseteB, DriveConstants.ramseteZeta),
            this.getFeedForward(),
            this.getKinematics(),
            this::getWheelSpeeds,
            this.getLeftController(),
            this.getRightController(),
            (leftVolts, rightVolts) -> {
                System.out.println("l volts: " + leftVolts + " | r volts: " + rightVolts);
                this.tankDriveVolts(-leftVolts, -rightVolts);
            },
            this
        ).andThen(()->this.tankDrive(0, 0));
    }
    public Command DriveTrajectoryToHub(double hubDistance) {
        TrajectoryConfig config = new TrajectoryConfig(DriveConstants.vVoltSecondsPerMeter, DriveConstants.aVoltSecondsSquaredPerMeter);
        
        Translation2d hubDisplacement = getPose().getTranslation().minus(FieldConstants.hubCenter);
        double angle = Math.atan(hubDisplacement.getY()/hubDisplacement.getX());
        Translation2d normalizedDisplacement = hubDisplacement.times(
            1 / Math.sqrt(
                Math.pow(hubDisplacement.getX(), 2) + 
                Math.pow(hubDisplacement.getY(), 2)
            )
        );
        Pose2d end = new Pose2d(normalizedDisplacement.times(hubDistance), Rotation2d.fromDegrees(angle * 180./Math.PI));
        
        return PathCommand(
            TrajectoryGenerator.generateTrajectory(
                this.getPose(), 
                null, 
                end,
                config
            ), 
            false
        );
    }
    public Command DriveLineToHub(double hubDistance) {
        Translation2d hubDisplacement = getPose().getTranslation().minus(FieldConstants.hubCenter);
        double angle = Math.atan(hubDisplacement.getY()/hubDisplacement.getX());

        DoubleSupplier distanceSupplier = () -> {
            return Math.sqrt(
                Math.pow(getPose().getTranslation().minus(FieldConstants.hubCenter).getX(), 2) + 
                Math.pow(getPose().getTranslation().minus(FieldConstants.hubCenter).getY(), 2)
            );
        };

        PIDController turnController = new PIDController(
            DriveConstants.driveP, 
            DriveConstants.driveI,
            DriveConstants.driveD
        );
        turnController.enableContinuousInput(0, 180);
        turnController.setTolerance(2);
        return new SequentialCommandGroup(
            new PIDCommand(
                turnController,
                this::getHeading, 
                ()->angle,
                (output)->{
                    this.arcadeDrive(
                        Math.abs(distanceSupplier.getAsDouble()) > 0.2 ? (
                            distanceSupplier.getAsDouble() > 0 ? 1 : -1
                        ) : 0,
                        output
                    );
                },
                this
            ),
            new PIDCommand(
                getLeftController(),
                distanceSupplier, 
                ()->hubDistance,
                (output)->{
                    this.arcadeDrive(
                        0,
                        output
                    );
                },
                this
            )
            /**
             * Add a vision align PID over here to fine tune shots if needed
             */
        );
    }

    /* CONVERSIONS */
    public static double nativePositionToMeters(double nativeUnits) { 
        // Helper function to convert native units (encoder counts) to meters for odometry
        double wheelRotations = TalonFXConversions.Native2Rotations(nativeUnits) * DriveConstants.gearboxRatio;
        return wheelRotations * (Math.PI * DriveConstants.wheelDiameter);
    }

    public static double metersToNativePosition(double meters) { 
        // Helper function to convert from meters to native encoder units for PID'ing
        double wheelRotations = meters / (Math.PI * DriveConstants.wheelDiameter);
        double motorRotations = wheelRotations / DriveConstants.gearboxRatio;
        return TalonFXConversions.Rotations2Native(motorRotations);
    }
    
    public static double metersPerSecondToNative(double metersPerSecond) { 
        // Helper function to convert from meters per seconds to encoder units per epoch for PID'ing
        double wheelRPS = metersPerSecond / (Math.PI * DriveConstants.wheelDiameter);
        double motorRPS = wheelRPS * DriveConstants.gearboxRatio;
        return TalonFXConversions.RPM2Native(motorRPS / 60);
    }

    public static double nativeToMetersPerSecond(double nativeUnits) { 
        // Helper function to convert from meters per seconds to encoder units per epoch for PID'ing
        double motorRPS = TalonFXConversions.Native2RPM(nativeUnits) * 60 / (double)DriveConstants.gearboxRatio;
        return motorRPS * (Math.PI * DriveConstants.wheelDiameter);
    }

    /* BOILERPLATE */
    /* PID Getters */
    public PIDController getLeftController() { return leftController; }
    public PIDController getRightController() { return rightController; }
    public SimpleMotorFeedforward getFeedForward() { return feedforward; }

    /* Sensor Getters */
    public double getLeftEncoderPosition() { 
        return nativePositionToMeters((frontLeft.getSelectedSensorPosition() + backLeft.getSelectedSensorPosition())/2.);
    }
    public double getLeftEncoderVelocity() {
        return nativeToMetersPerSecond((frontLeft.getSelectedSensorVelocity() + backLeft.getSelectedSensorVelocity())/2. * 10);
    }

    public double getRightEncoderPosition() { 
        return nativePositionToMeters((frontRight.getSelectedSensorPosition() + backRight.getSelectedSensorPosition())/2.); 
    }
    public double getRightEncoderVelocity() {   
        return nativeToMetersPerSecond((frontRight.getSelectedSensorVelocity() + backRight.getSelectedSensorVelocity())/2. * 10); 
    }
    
    public double getHeading() {
        return DriveConstants.invertGyro ? -gyro.getAngle() : gyro.getAngle();
    }
    
    public double getHeadingRate() { return DriveConstants.invertGyro ? -gyro.getRate() : gyro.getRate(); }

    /* Odometry Helper Functions */
    public DifferentialDriveKinematics getKinematics() { return kinematics; }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() { return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getLeftEncoderVelocity()); }
    public Pose2d getPose() { return odometry.getPoseMeters(); /*odometry.getEstimatedPosition();*/ }
}