package frc.robot.utils.tunables;

public abstract class Tunable<T> {
    private static final String tableKey = "Tunables";
    protected T defaultValue;
    protected T lastHasChangedValue = defaultValue;
    protected String key;
    public Tunable (String key){ this.key = tableKey + "/" + key; }

    public Tunable (String key, T defaultValue){
        this.key = tableKey + "/" + key;
        this.defaultValue = defaultValue;
    }

    public T getDefault() { return defaultValue; }
    public abstract void setDefault(T defaultValue);
    public abstract T get();

    public boolean hasChanged() {
        T currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }
        return false;
    }
}