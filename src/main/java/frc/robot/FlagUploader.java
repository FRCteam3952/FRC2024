package frc.robot;

import frc.robot.util.NetworkTablesUtil;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public class FlagUploader {
    public FlagUploader() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }
    /**
     * This method is used to upload the contents of the {@link frc.robot.Flags Flags} class to NetworkTables.
     * This is useful for debugging since the flags control robot functionality and allow an uploaded method to quickly check which parts of the robot code are enabled or disabled.
     */
    public static void uploadFlagsClass() {
        try {
            Class<Flags> clazz = Flags.class;
            var flagsTable = NetworkTablesUtil.MAIN_ROBOT_TABLE.getSubTable("Flags");

            for (Field field : clazz.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers())) {
                    try {
                        flagsTable.getEntry(field.getName()).setValue(field.get(null));
                    } catch (IllegalAccessException e) {
                        System.out.println("Unable to upload value of Flags field " + field.getName());
                    }
                }
            }

            for (Class<?> c : clazz.getClasses()) {
                var subTable = flagsTable.getSubTable(c.getSimpleName());
                for (Field field : c.getDeclaredFields()) {
                    if (Modifier.isStatic(field.getModifiers())) {
                        try {
                            subTable.getEntry(field.getName()).setValue(field.get(null));
                        } catch (IllegalAccessException e) {
                            System.out.println("Unable to upload value from Flags subclass " + c.getName() + ", field " + field.getName());
                        }
                    }
                }
            }
        } catch (Exception e) {
            System.out.println("error while uploading the flags classes:");
            e.printStackTrace();
        }
    }
}