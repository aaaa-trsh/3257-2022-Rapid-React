package frc.robot.utils.tunables;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TunableNumberArray extends Tunable<double[]> {

    public TunableNumberArray(String key) {
        super(key);
        setDefault(defaultValue);
    }

    public TunableNumberArray(String key, double[] defaultValue) {
        super(key, defaultValue);
        setDefault(defaultValue);
    }

    @Override
    public void setDefault(double[] defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.tuning) { SmartDashboard.putNumberArray(key, SmartDashboard.getNumberArray(key, defaultValue)); } 
        else { SmartDashboard.delete(key); }
    }

    @Override
    public double[] get() {
        return Constants.tuning ? SmartDashboard.getNumberArray(key, defaultValue) : defaultValue;
    }

    @Override
    public boolean hasChanged() {
        double[] currentValue = get();
        if (!Arrays.equals(currentValue, lastHasChangedValue)) {
            lastHasChangedValue = currentValue;
            return true;
        }
        return false;
    }
}