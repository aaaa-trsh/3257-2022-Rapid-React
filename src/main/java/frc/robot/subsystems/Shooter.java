package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.TunableNumber;
import frc.robot.utils.Conversions.TalonFXConversions;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX lowerFlywheel = new WPI_TalonFX(ShooterConstants.lowerFlywheelPort);
    private WPI_TalonFX upperFlywheel = new WPI_TalonFX(ShooterConstants.upperFlywheelPort);
    
    // private final TunableNumber maxVel = new TunableNumber("Shooter/MaxRPM");
    // private final TunableNumber maxAccel = new TunableNumber("Shooter/MaxAccelRPMPerSec2");
    // private final TunableNumber maxJerk = new TunableNumber("Shooter/MaxJerkRPMPerSec3");
    private final TunableNumber p = new TunableNumber("Shooter/P");
    private final TunableNumber i = new TunableNumber("Shooter/I");
    private final TunableNumber d = new TunableNumber("Shooter/D");
    private final TunableNumber f = new TunableNumber("Shooter/F");
    private final TunableNumber tolerance = new TunableNumber("Shooter/ToleranceRPM");
    
    private double lowerRPM, upperRPM;

    public Shooter() {
        // maxVel.setDefault(2650.0);
        // maxAccel.setDefault(2000.0);
        // maxJerk.setDefault(2500.0);
        p.setDefault(0.14);
        i.setDefault(0.0);
        d.setDefault(0.1);
        f.setDefault(0.5);
        tolerance.setDefault(0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.nominalOutputForward = 0;
        config.nominalOutputReverse = 0;
        config.peakOutputForward = 1;
        config.peakOutputReverse = -1;
        config.slot0.kP = p.get();
        config.slot0.kI = i.get();
        config.slot0.kD = d.get();
        config.slot0.kF = f.get();
        // config.closedloopRamp = 0.1;
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
    
    public void setSpeedsFromDistance() {

    }

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
            System.out.println(
                " P: " + p.get() + 
                " I: " + i.get() + 
                " D: " + d.get() + 
                " F: " + f.get()
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
}
