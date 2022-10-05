package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;
import frc.robot.utils.Conversions.TalonFXConversions;
import frc.robot.utils.interpolables.Interpolable.InterpolatingTreeMap;
import frc.robot.utils.interpolables.TupleInterpolable;
import frc.robot.utils.tunables.TunableNumber;
import frc.robot.utils.tunables.TunableNumberArray;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX lowerFlywheel = new WPI_TalonFX(ShooterConstants.lowerFlywheelPort);
    private WPI_TalonFX upperFlywheel = new WPI_TalonFX(ShooterConstants.upperFlywheelPort);
    
    private Limelight limelight = new Limelight("limelight-shooter");

    private TunableNumber p = new TunableNumber("Shooter/P", ShooterConstants.P);
    private TunableNumber i = new TunableNumber("Shooter/I", ShooterConstants.I);
    private TunableNumber d = new TunableNumber("Shooter/D", ShooterConstants.D);
    private TunableNumber f = new TunableNumber("Shooter/F", ShooterConstants.F);
    
    private TunableNumberArray distanceLandmarks = new TunableNumberArray("Shooter/DistanceLandmarks", new double[]{0, -5, -10, -12, -15, -17, -20});
    private TunableNumberArray baseSpeedLandmarks = new TunableNumberArray("Shooter/BaseSpeedLandmarks", new double[]{1150, 1100, 1150, 1150, 1200, 1250, 1500});
    private TunableNumberArray ratioLandmarks = new TunableNumberArray("Shooter/RatioLandmarks", new double[]{0.15, 0.3, 0.3, 0.2, 0.25, 0.15, 0.15});

    private TunableNumber shooterFineOffset = new TunableNumber("Shooter/DistanceOffset", 0.);

    private InterpolatingTreeMap<Pair<Double, Double>> interpolatingTreemap = new InterpolatingTreeMap<>();
    
    private double lowerRPM, upperRPM;

    public Shooter() {
        updateTreeMap();
        setLightState(false);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.nominalOutputForward = 0;
        config.nominalOutputReverse = 0;
        config.peakOutputForward = 1;
        config.peakOutputReverse = -1;
        config.closedloopRamp = 0.7;
        config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        // lower flywheel config
        lowerFlywheel.configFactoryDefault();
        lowerFlywheel.setInverted(InvertType.InvertMotorOutput);
        lowerFlywheel.setNeutralMode(NeutralMode.Coast);
        lowerFlywheel.configAllSettings(config);

        // upper flywheel config
        upperFlywheel.configFactoryDefault();
        upperFlywheel.setInverted(InvertType.None);
        upperFlywheel.setNeutralMode(NeutralMode.Coast);
        upperFlywheel.configAllSettings(config);

        updateFlywheelPIDs();
    }

    private void updateTreeMap() {
        var interpolatingTreemap = new InterpolatingTreeMap<Pair<Double, Double>>();
        double[] dist = distanceLandmarks.get();
        double[] baseSpeeds = baseSpeedLandmarks.get();
        double[] ratios = ratioLandmarks.get();

        for (int i = 0; i < dist.length; i++) {
            interpolatingTreemap.put(dist[i], new TupleInterpolable(baseSpeeds[i], ratios[i]));
        }
        this.interpolatingTreemap = interpolatingTreemap;
        
        System.out.println(
            distanceLandmarks.hasChanged() + " " + 
            baseSpeedLandmarks.hasChanged() + " " + 
            ratioLandmarks.hasChanged() + " "
        );
    }

    private void updateFlywheelPIDs() {
        lowerFlywheel.config_kP(0, p.get(), Constants.timeout);
        lowerFlywheel.config_kI(0, i.get(), Constants.timeout);
        lowerFlywheel.config_kD(0, d.get(), Constants.timeout);
        lowerFlywheel.config_kF(0, f.get(), Constants.timeout);

        upperFlywheel.config_kP(0, p.get(), Constants.timeout);
        upperFlywheel.config_kI(0, i.get(), Constants.timeout);
        upperFlywheel.config_kD(0, d.get(), Constants.timeout);
        upperFlywheel.config_kF(0, f.get(), Constants.timeout);
    }

    public void setLightState(boolean on) {
        limelight.setLightState(on ? 3 : 1);
    }
    
    public void setShooterSpeeds(double lowerRPM, double upperRPM) {
        this.lowerRPM = lowerRPM;
        this.upperRPM = upperRPM;
    }

    public void setShooterFromDistance(double distance) {
        var shooterSpeeds = interpolatingTreemap.interpolate(distance + shooterFineOffset.get());
        setShooterSpeeds(
            (shooterSpeeds.getFirst() * (1. - shooterSpeeds.getSecond())), 
            (shooterSpeeds.getFirst() * shooterSpeeds.getSecond())
        );
        System.out.println(shooterSpeeds.getFirst() + " | " + shooterSpeeds.getSecond());
    }

    public void setShooterFromDistance() { setShooterFromDistance(getLimelight().getPitchError()); }

    public Pair<Double, Double> getFlywheelNativeVelocityError() { return new Pair<Double, Double>(lowerFlywheel.getClosedLoopError(), upperFlywheel.getClosedLoopError()); }

    @Override
    public void periodic() {
        if (p.hasChanged() | i.hasChanged() | d.hasChanged() | f.hasChanged()) {
            updateFlywheelPIDs();
        }

        if (distanceLandmarks.hasChanged() | baseSpeedLandmarks.hasChanged() | ratioLandmarks.hasChanged()) {
            updateTreeMap();
        }
        
        if (lowerRPM > 0 | upperRPM > 0) {
            lowerFlywheel.set(TalonFXControlMode.Velocity, TalonFXConversions.RPM2Native(lowerRPM));
            upperFlywheel.set(TalonFXControlMode.Velocity, TalonFXConversions.RPM2Native(upperRPM));
        } else {
            lowerFlywheel.set(TalonFXControlMode.PercentOutput, 0);
            upperFlywheel.set(TalonFXControlMode.PercentOutput, 0);
        }

        // lowerFlywheel.set(TalonFXControlMode.PercentOutput, lowerRPM);
        // upperFlywheel.set(TalonFXControlMode.PercentOutput, upperRPM);
        
        SmartDashboard.putNumber("lower flywheel rpm", TalonFXConversions.Native2RPM(lowerFlywheel.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("upper flywheel rpm", TalonFXConversions.Native2RPM(upperFlywheel.getSelectedSensorVelocity()));

        SmartDashboard.putNumber("lower flywheel target", TalonFXConversions.RPM2Native(lowerRPM));
        SmartDashboard.putNumber("upper flywheel target", TalonFXConversions.RPM2Native(upperRPM));
    }

    public Limelight getLimelight() { return limelight; }

    public Command spinUpWithVisionCommand() {
        return new InstantCommand(() -> {
            if (getLimelight().hasTarget()) { setShooterFromDistance(); }
            else { setShooterFromDistance(-4); }
        }, this);
    }
    public Command spinUpForTuningCommand() {
        return new InstantCommand(() -> {
            if (getLimelight().hasTarget()) { setShooterFromDistance(); }
            else { setShooterFromDistance(-4); }
        }, this);
    }
    public Runnable spinUpOverrideCommand() { return () -> setShooterFromDistance(-4); }
    public Runnable spinDownCommand() { return () -> setShooterSpeeds(0, 0); }

}
