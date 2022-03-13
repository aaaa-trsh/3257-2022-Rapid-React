package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntestineConstants;

public class Intestines extends SubsystemBase {    
    private CANSparkMax magazineMotor = new CANSparkMax(IntestineConstants.magazinePort, MotorType.kBrushless);
    private DigitalInput magazineSwitch = new DigitalInput(IntestineConstants.magazineSwitchPort);

    public Intestines() { }

    public boolean isBallInQueue() { return !magazineSwitch.get(); }
    public void setMagazinePercent(double percent) { magazineMotor.set(percent); }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Magazine Switch", isBallInQueue());
    }
}
