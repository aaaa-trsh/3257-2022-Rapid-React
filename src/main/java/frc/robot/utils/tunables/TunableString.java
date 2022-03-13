package frc.robot.utils.tunables;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TunableString extends Tunable<String> {

    public TunableString(String key) {
        super(key);
    }

    public TunableString(String key, String defaultValue) {
        super(key, defaultValue);
    }

    @Override
    public void setDefault(String defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.tuning) { SmartDashboard.putString(key, SmartDashboard.getString(key, defaultValue)); } 
        else { SmartDashboard.delete(key); }
    }

    @Override
    public String get() {
        return Constants.tuning ? SmartDashboard.getString(key, defaultValue) : defaultValue;
    }
}