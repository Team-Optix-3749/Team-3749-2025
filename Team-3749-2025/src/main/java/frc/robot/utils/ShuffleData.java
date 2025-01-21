package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Stores data that can be seen/edited in ShuffleBoard
 * 
 * @author Rohin Sood
 * 
 * @param <T> Type of data: Supported types include String, Double, and Boolean
 */
public class ShuffleData<T> {
    private T defaultVal;
    private ShuffleboardTab tab;
    private GenericEntry entry;
    private Map<Integer, T> lastHasChangedVals = new HashMap<>();

    /**
     * Creates a new SmartData instance
     * 
     * @param tab        Tab on ShuffleBoard
     * @param name       Key on ShuffleBoard
     * @param defaultVal Default value
     */
    public ShuffleData(String tab, String name, T defaultVal) {
        this.defaultVal = defaultVal;
        this.tab = Shuffleboard.getTab(tab);
        this.entry = this.tab.add(name, defaultVal).getEntry();
        entry.setValue(defaultVal);
    }

    public void setDefault(T defaultVal) {
        this.defaultVal = defaultVal;
    }

    @SuppressWarnings("unchecked")
    public T get() {
        NetworkTableValue value = entry.get();

        if (value.getType() == NetworkTableType.kBoolean) {
            return (T) (Boolean) value.getBoolean();

        } else if (value.getType() == NetworkTableType.kDouble) {
            return (T) (Double) value.getDouble();

        } else if (value.getType() == NetworkTableType.kString) {
            return (T) (String) value.getString();

        } else {
            return this.defaultVal;
        }
    }

    /**
     * checks whether the number has changed since the last time this method has
     * been called
     * 
     * @param id Unique identified for the called to avoid conflicts shared between
     *           multiple instances. Recommended to pass in the result of
     *           'Obj.hashCode()'
     * @return true if the value has changed for its last check
     */
    public boolean hasChanged(int id) {
        T currentVal = get();
        T lastVal = lastHasChangedVals.get(id);

        if (lastVal == null || !currentVal.equals(lastVal)) {
            lastHasChangedVals.put(id, currentVal);
            return true;
        }

        return false;
    }

    public void set(T val) {
        entry.setValue(val);
    }

    public static void put(String tabName, Sendable sendableData) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        tab.add(sendableData);
    }

    public static void put(String tabName, String key, Object data) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        tab.add(key, data);
    }
}