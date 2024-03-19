package frc.robot;

/**
 * Control flags for the robot.
 * These flags should be changed to enable or disable functionality from a centralized location rather than digging through robot code.
 * <p><br>
 * REMEMBER TO RESET THESE BEFORE COMPETITION GAME BEGINS UNLESS YOU ARE ABSOLUTELY CERTAIN YOU WANT IT DISABLED.
 */
public final class Flags {
    /**
     * Flags relating to the Operator console (i.e. relating to the Driver Station laptop).
     */
    public static class Operator {
        public static final boolean USING_NINTENDO_SWITCH_CONTROLLER = true;
    }

    /**
     * Flags relating to the drive train.
     */
    public static class DriveTrain {
        /**
         * Whether the drive train is physically attached and existing. If false, no motor controllers are initialized since they are assumed to be nonexistent.
         */
        public static final boolean IS_ATTACHED = false;

        /**
         * Whether the drive train should be allowed to send power to motor controllers. If false, motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLED = false;

        /**
         * Whether the drive motors should be allowed to run. If false, drive motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLE_DRIVE_MOTORS = true;
        /**
         * Whether the turn motors should be allowed to run. If false, turn motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLE_TURN_MOTORS = true;

        /**
         * Whether the drive motors should be allowed to run based on PID control. If false, drive motors can only be run by sending raw power values/voltages.
         */
        public static final boolean DRIVE_PID_CONTROL = true;
        /**
         * Whether the turn motors should be allowed to run based on PID control. If false, drive motors can only be run by sending raw power values/voltages.
         */
        public static final boolean TURN_PID_CONTROL = true;

        /**
         * Whether to lower the maximum speed of the drive train.
         */
        public static final boolean LOWER_MAX_SPEED = false;

        /**
         * Whether to enable optimizing swerve module rotations.
         */
        public static final boolean SWERVE_MODULE_OPTIMIZATION = true;
        /**
         * Whether to only optimize swerve modules if the speeds are below a certain value.
         */
        public static final boolean SPEED_BASED_SWERVE_MODULE_OPTIMIZATION = false;

        /**
         * Whether to enable the test drive command rather than the normal command.
         */
        public static final boolean USE_TEST_DRIVE_COMMAND = false;
    }

    /**
     * Flags relating to the intake.
     */
    public static class Intake {
        /**
         * Whether the intake is physically attached and existing. If false, no motor controllers are initialized since they are assumed to be nonexistent.
         */
        public static final boolean IS_ATTACHED = false;

        /**
         * Whether the intake should be allowed to send power to motor controllers. If false, motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLED = true;

        /**
         * Whether the pivot is enabled.
         */
        public static final boolean PIVOT_ENABLED = false;

        /**
         * Whether the intake pivot motors should be allowed to run based on PID control. If false, the pivot motors can only be run by sending raw power values/voltages.
         */
        public static final boolean PIVOT_PID_CONTROL = true;

        /**
         * Whether to enable the test intake command rather than the normal command.
         */
        public static final boolean USE_TEST_INTAKE_COMMAND = false;
    }

    /**
     * Flags relating to the conveyor.
     */
    public static class Conveyor {
        /**
         * Whether the conveyor is physically attached and existing. If false, no motor contorllers are initialized since they are assumed to be nonexistent.
         */
        public static final boolean IS_ATTACHED = false;

        /**
         * Whether the conveyor should be allowed to send power to motor controllers. If false, motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLED = true;
    }

    /**
     * Flags relating to the shooter.
     */
    public static class Shooter {
        /**
         * Whether the shooter is physically attached and existing. If false, no motor controllers are initialized since they are assumed to be nonexistent.
         */
        public static final boolean IS_ATTACHED = true;

        /**
         * Whether the shooter should be allowed to send power to motor controllers. If false, motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLED = true;

        /**
         * Whether the pivot is enabled.
         */
        public static final boolean PIVOT_ENABLED = true;

        /**
         * Whether the shooter pivot motors should be allowed to run based on PID control. If false, the pivot motors can only be run by sending raw power values/voltages.
         */
        public static final boolean PIVOT_PID_CONTROL = true;

        /**
         * Whether the shooter motors should use PID to target a specific RPM.
         */
        public static final boolean SHOOTER_RPM_PID_CONTROL = true;

        /**
         * Whether to enable the test shooter command rather than the normal command.
         */
        public static final boolean USE_TEST_SHOOTER_COMMAND = true;
    }

    /**
     * Flags relating to the climber.
     */
    public static class Climber {
        /**
         * Whether the climber is physically attached and existing. If false, no motor controllers are initialized since they are assumed to be nonexistent.
         */
        public static final boolean IS_ATTACHED = false;

        /**
         * Whether the climber should be allowed to send power to motor controllers. If false, motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLED = false;
    }
}