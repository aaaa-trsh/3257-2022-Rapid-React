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

public class Shooter extends SubsystemBase {
    private WPI_TalonFX lowerFlywheel = new WPI_TalonFX(ShooterConstants.lowerFlywheelPort);
    private WPI_TalonFX upperFlywheel = new WPI_TalonFX(ShooterConstants.upperFlywheelPort);
    
    private Limelight limelight = new Limelight("limelight-bottom");

    private TunableNumber p = new TunableNumber("Shooter/P", ShooterConstants.P);
    private TunableNumber i = new TunableNumber("Shooter/I", ShooterConstants.I);
    private TunableNumber d = new TunableNumber("Shooter/D", ShooterConstants.D);
    private TunableNumber f = new TunableNumber("Shooter/F", ShooterConstants.F);
    
    private TunableNumberArray[] interpolatingTreemapLandmarks = { 
        new TunableNumberArray("Shooter/DistanceLandmarks", new double[]{6, -3, -12}),
        new TunableNumberArray("Shooter/LowerFlywheelLandmarks", new double[]{330, 240, 120}),
        new TunableNumberArray("Shooter/UpperFlywheelLandmarks", new double[]{770, 960, 1190}),
    };

    private InterpolatingTreeMap<Pair<Double, Double>> interpolatingTreemap = new InterpolatingTreeMap<>();
    
    private double lowerRPM, upperRPM;

    public Shooter() {
        var interpolatingTreemap = new InterpolatingTreeMap<Pair<Double, Double>>();
        double[] dist = interpolatingTreemapLandmarks[0].get();
        double[] low = interpolatingTreemapLandmarks[1].get();
        double[] up = interpolatingTreemapLandmarks[2].get();

        for (int i = 0; i < dist.length; i++) {
            interpolatingTreemap.put(dist[i], new TupleInterpolable(low[i], up[i]));
        }
        this.interpolatingTreemap = interpolatingTreemap;

        limelight.setLightState(0);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.nominalOutputForward = 0;
        config.nominalOutputReverse = 0;
        config.peakOutputForward = 1;
        config.peakOutputReverse = -1;
        config.slot0.kP = p.get();
        config.slot0.kI = i.get();
        config.slot0.kD = d.get();
        config.slot0.kF = f.get();
        config.closedloopRamp = ShooterConstants.rampRate;
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
    }
    
    public void setShooterSpeeds(double lowerRPM, double upperRPM) {
        this.lowerRPM = lowerRPM;
        this.upperRPM = upperRPM;
    }

    public void setShooterFromDistance(double distance) {
        var shooterSpeeds = interpolatingTreemap.interpolate(distance);
        System.out.println(shooterSpeeds.getFirst() + " | " + shooterSpeeds.getSecond());
        setShooterSpeeds(shooterSpeeds.getFirst()+50, shooterSpeeds.getSecond()+50);
    }

    public void setShooterFromDistance() { setShooterFromDistance(getLimelight().getPitchError()); }

    public Pair<Double, Double> getFlywheelNativeVelocityError() { return new Pair<Double, Double>(lowerFlywheel.getClosedLoopError(), upperFlywheel.getClosedLoopError()); }

    @Override
    public void periodic() {
        if (p.hasChanged() | i.hasChanged() | d.hasChanged() | f.hasChanged()) {
            lowerFlywheel.config_kP(0, p.get(), Constants.timeout);
            lowerFlywheel.config_kI(0, i.get(), Constants.timeout);
            lowerFlywheel.config_kD(0, d.get(), Constants.timeout);
            lowerFlywheel.config_kF(0, f.get(), Constants.timeout);

            upperFlywheel.config_kP(0, p.get(), Constants.timeout);
            upperFlywheel.config_kI(0, i.get(), Constants.timeout);
            upperFlywheel.config_kD(0, d.get(), Constants.timeout);
            upperFlywheel.config_kF(0, f.get(), Constants.timeout);
            // System.out.println(
            //     " P: " + p.get() + 
            //     " I: " + i.get() + 
            //     " D: " + d.get() + 
            //     " F: " + f.get()
            // );
        }

        if (interpolatingTreemapLandmarks[0].hasChanged() | interpolatingTreemapLandmarks[1].hasChanged() | interpolatingTreemapLandmarks[2].hasChanged()) {
            var interpolatingTreemap = new InterpolatingTreeMap<Pair<Double, Double>>();
            double[] dist = interpolatingTreemapLandmarks[0].get();
            double[] low = interpolatingTreemapLandmarks[1].get();
            double[] up = interpolatingTreemapLandmarks[2].get();

            for (int i = 0; i < dist.length; i++) {
                interpolatingTreemap.put(dist[i], new TupleInterpolable(low[i], up[i]));
            }
            this.interpolatingTreemap = interpolatingTreemap;
            System.out.println(
                interpolatingTreemapLandmarks[0].hasChanged() + " " + 
                interpolatingTreemapLandmarks[1].hasChanged() + " " + 
                interpolatingTreemapLandmarks[2].hasChanged() + " "
            );
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
}
