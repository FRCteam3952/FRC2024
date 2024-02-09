package frc.robot;

/**
 * Control flags for the robot.
 * These flags should be changed to enable or disable functionality rather than digging through robot code.
 * <p><br>
 * REMEMBER TO RESET THESE BEFORE COMPETITION GAME BEGINS UNLESS YOU ARE ABSOLUTELY CERTAIN YOU WANT IT DISABLED.
 */
public final class Flags {
    public static class DriveTrain {
        public static final boolean ENABLED = true;

        public static final boolean ENABLE_DRIVE_MOTORS = true;
        public static final boolean ENABLE_TURN_MOTORS = true;

        public static final boolean DRIVE_PID_CONTROL = true;
        public static final boolean TURN_PID_CONTROL = true;

        public static final boolean LOWER_MAX_SPEED = false;

        public static final boolean SWERVE_MODULE_OPTIMIZATION = false;
        public static final boolean SPEED_BASED_SWERVE_MODULE_OPTIMIZATION = false;

        public static final boolean USE_TEST_DRIVE_COMMAND = false;
    }
}
