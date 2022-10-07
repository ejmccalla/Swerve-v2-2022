package frc.robot;

/**
 * The Constants class constains all of the port mappings and other numerical values which don't
 * require calibration.
 */
public final class Constants {

    /**
     * The Hardware constants are port numbers and/or CAN ID's of the robot hardware which don't
     * belong to an explicit sub-system.
     */
    public static final class Hardware {
        public static final int PDP_ID = 0;
        public static final int PCM_ID = 0;
    }

    /**
     * The Drivetrain constants are port numbers and/or CAN ID's of the robot hardware which don't
     * belong to an explicit sub-system.
     */
    public static final class Drivetrain {
        public static final int FRONT_LEFT_TURN = 0;
        public static final int FRONT_LEFT_DRIVE = 0;
        public static final int FRONT_LEFT_QUAD_A_DIO_CHANNEL = 0;
        public static final int FRONT_LEFT_QUAD_B_DIO_CHANNEL = 0;
        public static final int FRONT_LEFT_PWM_DIO_CHANNEL = 0;
        public static final int FRONT_RIGHT_TURN = 0;
        public static final int FRONT_RIGHT_DRIVE = 0;
        public static final int FRONT_RIGHT_QUAD_A_DIO_CHANNEL = 0;
        public static final int FRONT_RIGHT_QUAD_B_DIO_CHANNEL = 0;
        public static final int FRONT_RIGHT_PWM_DIO_CHANNEL = 0;
        public static final int REAR_LEFT_TURN = 0;
        public static final int REAR_LEFT_DRIVE = 0;
        public static final int REAR_LEFT_QUAD_A_DIO_CHANNEL = 0;
        public static final int REAR_LEFT_QUAD_B_DIO_CHANNEL = 0;
        public static final int REAR_LEFT_PWM_DIO_CHANNEL = 0;
        public static final int REAR_RIGHT_TURN = 0;
        public static final int REAR_RIGHT_DRIVE = 0;
        public static final int REAR_RIGHT_QUAD_A_DIO_CHANNEL = 0;
        public static final int REAR_RIGHT_QUAD_B_DIO_CHANNEL = 0;
        public static final int REAR_RIGHT_PWM_DIO_CHANNEL = 0;

        // https://www.swervedrivespecialties.com/products/mk4-swerve-module#:~:text=The%20steering%20gear%20ratio%20of,4%20different%20drive%20gear%20ratios.
        public static final double TURN_GEAR_RATIO = 12.8;
        public static final double DRIVE_GEAR_RATIO = 6.75;


    }

}
