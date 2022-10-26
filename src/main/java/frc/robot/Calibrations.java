package frc.robot;

/**
 * The Calibrations class constains all of the robot calibrations and measurement values.
 */
public final class Calibrations {

    /** 
     * Average wheel diameter (best: calibrate with encoders, ok: use tape measure).
     *
     * @implNote This should be done before each event minimum.
     */
    public static final double WHEEL_DIAMETER_INCH = 4.0;
    
    /**
     * Distance between right and left wheels (best: calibrate with encoders, ok: use tape measure).
     *
     * @implNote This should be done before each event minimum.
     */
    public static final double TRACK_WIDTH_INCH = 19.5;

    /**
     * Distance between front and back wheels (best: calibrate with encoders, ok: use tape measure).
     *
     * @implNote This should be done before each event minimum.
     */
    public static final double WHEEL_BASE_INCH = 19.5;


    /**
     * The PWM encoder angle for zero'ing the wheel (best: place the robot against a wall, push the
     * robot along the wall for 10 feet, and take the average measured angle, ok: manually align
     * the wheels and take a single reading).
     *
     * <p>The ordering is: front left, front right, rear left, rear right
     *
     * @implNote The "ok" method should be done before each match minimum.
     */
    public static final double[] ZEROS_RAD = {1.27, 0.91, 0.27, 5.82};

    /**
     * The P-gain of the PID turning controller.
     *
     * @implNote This should be tuned before each event minimum.
     */
    public static final double TURN_P_GAIN = 2.5;

    /**
     * The D-gain of the PID turning controller.
     *
     * @implNote This should be tuned before each event minimum.
     */
    public static final double TURN_D_GAIN = 0.0;

    /**
     * The static gain of the PID FF turning controller.
     *
     * @implNote This should be tuned before each event minimum.
     */
    public static final double TURN_FF_KS_GAIN = 0.0;

    /**
     * The velocity gain of the PID FF turning controller.
     *
     * @implNote This should be tuned before each event minimum.
     */
    public static final double TURN_FF_KV_GAIN = 0.0;

    /**
     * The maximum turning controller error considered on-target.
     *
     * @implNote This should be tuned before each event minimum.
     */
    public static final double MAX_TURN_ERROR_DEG = 1.0;

    /**
     * The P-gain of the PID driving controller.
     *
     * @implNote This should be tuned before each event minimum.
     */
    public static final double DRIVE_P_GAIN = 0.0;

    /**
     * The D-gain of the PID driving controller.
     *
     * @implNote This should be tuned before each event minimum.
     */
    public static final double DRIVE_D_GAIN = 0.0;

    /**
     * The static gain of the PID FF driving controller.
     *
     * @implNote This should be tuned before each event minimum.
     */
    public static final double DRIVE_FF_KS_GAIN = 0.0;

    /**
     * The velocity gain of the PID FF driving controller.
     *
     * @implNote This should be tuned before each event minimum.
     */
    public static final double DRIVE_FF_KV_GAIN = 0.0;

    /**
     * The maximum angular velocity used for constraining trajectory profile.
     *
     * @implNote This should be increased towards maximum as drivers improve.
     */
    public static final double MAX_TURN_VELOCITY_RPS = 20 * Math.PI;

    /**
     * The maximum angular acceleration - used for constraining trajectory profile.
     *
     * @implNote This should be increased towards maximum as drivers improve.
     */
    public static final double MAX_TURN_ACCELERATION_RPSS = 20 * Math.PI;

    /**
     * The maximum drive velocity.
     *
     * @implNote This should be increased towards maximum as drivers improve.
     */
    public static final double MAX_DRIVE_VELOCITY_MPS = 2.5;

}
