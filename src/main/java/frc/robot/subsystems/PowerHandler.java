package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.util.NetworkTablesUtil;

import java.lang.reflect.Field;

public class PowerHandler {
    private static final DoublePublisher voltagePublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("battery_voltage").publish();
    private static final DoublePublisher currentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("total_current").publish();

    private final PowerDistribution pdp;

    public PowerHandler() {
        this.pdp = new PowerDistribution();

        this.pdp.clearStickyFaults();

        var f = pdp.getStickyFaults();
        System.out.println("PDP FAULTS: ");
        try {
            for (Field field : f.getClass().getDeclaredFields()) {
                // System.out.println(field.getName() + ": " + field.getBoolean(f));
            }
        } catch (Exception ignored) {
        }
    }

    public double getVoltage() {
        return this.pdp.getVoltage();
    }

    public double getTotalCurrent() {
        return this.pdp.getTotalCurrent();
    }

    public double getChannelCurrent(int channel) {
        return this.pdp.getCurrent(channel);
    }

    public void updateNT() {
        // voltagePublisher.set(this.getVoltage());
        // currentPublisher.set(this.getTotalCurrent());
    }
}
