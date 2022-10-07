package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.Limelight;
import frc.robot.utils.Conversions.TalonFXConversions;
import frc.robot.utils.tunables.TunableNumber;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX backLeft = new WPI_TalonFX(DriveConstants.backLeftPort);
    private WPI_TalonFX backRight = new WPI_TalonFX(DriveConstants.backRightPort);
    private WPI_TalonFX frontLeft = new WPI_TalonFX(DriveConstants.frontLeftPort);
    private WPI_TalonFX frontRight = new WPI_TalonFX(DriveConstants.frontRightPort);

    private DifferentialDrive differentialDrive = new DifferentialDrive((MotorController)frontLeft, (MotorController)frontRight);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackwidth);
    
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.sVolts, DriveConstants.vVoltSecondsPerMeter, DriveConstants.aVoltSecondsSquaredPerMeter);

    // private TunableNumber turnP = new TunableNumber("Drive/TurnP", 0.03);
    // private TunableNumber turnD = new TunableNumber("Drive/TurnD", 0.);
    private PIDController turnController = new PIDController(0.04, 0.0, 0.0);

    private PIDController visionPIDController = new PIDController(0.03, 0.06, 0);

    // private PIDController leftController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
    // private PIDController rightController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);

    private AHRS gyro = new AHRS();

    private Field2d field = new Field2d();

    public Drivetrain() {
        // Calibrate n reset the gyro
        gyro.calibrate();
        gyro.reset();

        turnController.enableContinuousInput(-180, 180);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.closedloopRamp = 0.1;
        config.openloopRamp = 0;
        config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        // Reset all the drivetrain controllers
        frontLeft.configFactoryDefault();
        frontLeft.setNeutralMode(NeutralMode.Brake);
        frontLeft.setInverted(InvertType.InvertMotorOutput);
        frontLeft.configAllSettings(config);

        backLeft.configFactoryDefault();
        backLeft.follow(frontLeft);
        backLeft.setNeutralMode(NeutralMode.Brake);
        backLeft.setInverted(InvertType.FollowMaster);
        backLeft.configAllSettings(config);
        
        frontRight.configFactoryDefault();
        frontRight.setNeutralMode(NeutralMode.Brake);
        frontRight.setInverted(InvertType.None);
        frontRight.configAllSettings(config);

        backRight.configFactoryDefault();
        backRight.follow(frontRight);
        backRight.setInverted(InvertType.FollowMaster);
        backRight.setNeutralMode(NeutralMode.Brake);
        backRight.configAllSettings(config);

        SmartDashboard.putData("Field", field);
    }
    public void resetGyro() { gyro.reset(); }

    private boolean lastTurnHeadingInit = false; 
    @Override
    public void periodic() {
        // super jank but gyro data doesnt register until first periodic so
        // if (!lastTurnHeadingInit) {lastTurnHeading = getHeading(); lastTurnHeadingInit = true;} 
        // System.out.println(Math.round(this.getHeading()) + " " + Math.round(lastTurnHeading) + " " + Math.round(100*(this.getHeading() - lastTurnHeading))/100 + "        " + turnController.calculate(this.getHeading(), lastTurnHeading));

        /*if (turnP.hasChanged() | 
        turnD.hasChanged()) {
            turnController.setP(turnP.get());
            turnController.setD(turnD.get());
        }*/

        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftPosition(), getRightPosition());
        field.setRobotPose(getPose());

        SmartDashboard.putData("field", field);
        SmartDashboard.putNumber("right pos", getRightPosition());
        SmartDashboard.putNumber("left pos", getLeftPosition());
        SmartDashboard.putNumber("gyro", getHeading());
    }

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
        differentialDrive.arcadeDrive(throttle, turn);
    }
    
    public void resetOdometry(Pose2d pose) { 
        odometry.resetPosition(pose, pose.getRotation()); 
        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
        gyro.reset();
    }

    private double lastTurnHeading = 0;
    public RunCommand pidArcadeDrive(
        DoubleSupplier throttleSupplier,
        DoubleSupplier turnSupplier
    ) {
        return new RunCommand(
            () -> {
                double turnInput = turnSupplier.getAsDouble();
                // if (Math.abs(turnInput) < 0.05) {
                //     turnInput = turnController.calculate(this.getHeading(), lastTurnHeading);
                // } else {
                //     lastTurnHeading = this.getHeading();
                // }
                arcadeDrive(
                    throttleSupplier.getAsDouble(),
                    turnInput
                );
            },
                this
            );
    }

    public RunCommand aimWithVisionCommand(
        Limelight shooterLimelight, 
        DoubleSupplier manualTurnSupplier, 
        DoubleSupplier manualThrottleSupplier
    ) {
        return new RunCommand(() -> {
            // allow normal turn until target acquired
            double turn = manualTurnSupplier.getAsDouble();
            if (shooterLimelight.hasTarget()) { // set turn output
                turn = visionPIDController.calculate(shooterLimelight.getYawError(), 0);
            }
            arcadeDrive(
                manualThrottleSupplier.getAsDouble(), // allow normal throttle
                Math.max(Math.min(turn, 1), -1) // clamp turn output -1 to 1
            );
        }, this);
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
    public PIDController getTurnController() { return turnController; }
    public PIDController getLeftController() { return null;/*leftController;*/ }
    public PIDController getRightController() { return null;/*rightController;*/ }
    public SimpleMotorFeedforward getFeedForward() { return feedforward; }

    /* Sensor Getters */
    public double getLeftPosition() { 
        return nativePositionToMeters((frontLeft.getSelectedSensorPosition() + backLeft.getSelectedSensorPosition())/2.);
    }
    public double getLeftVelocity() {
        return nativeToMetersPerSecond((frontLeft.getSelectedSensorVelocity() + backLeft.getSelectedSensorVelocity())/2. * 10);
    }

    public double getRightPosition() { 
        return nativePositionToMeters((frontRight.getSelectedSensorPosition() + backRight.getSelectedSensorPosition())/2.); 
    }
    public double getRightVelocity() {   
        return nativeToMetersPerSecond((frontRight.getSelectedSensorVelocity() + backRight.getSelectedSensorVelocity())/2. * 10); 
    }
    
    public double getHeading() { return DriveConstants.invertGyro ? -gyro.getAngle() : gyro.getAngle(); }
    public double getHeadingRate() { return DriveConstants.invertGyro ? -gyro.getRate() : gyro.getRate(); }

    /* Odometry Helper Functions */
    public DifferentialDrive getDifferentialDrive() { return differentialDrive; }
    public DifferentialDriveKinematics getKinematics() { return kinematics; }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() { return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getLeftVelocity()); }
    public Pose2d getPose() { return odometry.getPoseMeters(); /*odometry.getEstimatedPosition();*/ }
}