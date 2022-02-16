package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntestineConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intestines extends SubsystemBase {
    // private Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, IntestineConstants.intakeSolenoidPort);
    // private CANSparkMax intakeMotor = new CANSparkMax(IntestineConstants.intakeSparkPort, MotorType.kBrushless);
    private boolean isIntakeActuated = false;
    
    private CANSparkMax magazineMotor = new CANSparkMax(IntestineConstants.magazinePort, MotorType.kBrushless);
    private DigitalInput magazineSwitch = new DigitalInput(IntestineConstants.magazineSwitchPort);

    public Intestines() {

    }

    public void actuateIntake(boolean on) {
        // intakeSolenoid.set(on);
        isIntakeActuated = on;
    }

    // public void setIntakePercent(double percent) { intakeMotor.set(percent); }

    public boolean getIntakeState() { return isIntakeActuated; }

    public boolean isBallInQueue() { return !magazineSwitch.get(); }
    public void setMagazinePercent(double percent) { magazineMotor.set(percent); }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Magazine Switch", isBallInQueue());
    }
}
