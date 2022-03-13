package frc.robot.utils.tunables;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TunableNumber extends Tunable<Double> {

    public TunableNumber(String key) {
        super(key);
        setDefault(defaultValue);
    }

    public TunableNumber(String key, Double defaultValue) {
        super(key, defaultValue);
        setDefault(defaultValue);
    }

    @Override
    public void setDefault(Double defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.tuning) { SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue)); } 
        else { SmartDashboard.delete(key); }
    }

    @Override
    public Double get() {
        return Constants.tuning ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
    }
}