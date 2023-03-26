package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import frc.robot.Constants;

public class TunableNumber {

    private static final String tableKey = "Tunable Numbers";

    private final String key;
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedDashboardNumber dashboardNumber;
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    public TunableNumber(String dashboardKey) {
        this.key = tableKey + "/" + dashboardKey;
    }

    public TunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    public void initDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (Constants.tuningMode) {
                dashboardNumber = new LoggedDashboardNumber(key, defaultValue);
            }
        }
    }

    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return Constants.tuningMode ? dashboardNumber.get() : defaultValue;
        }
    }

    public boolean hasChanged(int id) {
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }
}
