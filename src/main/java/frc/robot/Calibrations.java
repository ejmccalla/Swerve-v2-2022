package frc.robot;

/**
 * The Calibrations class contains all of the robot calibration and measurement values.
 */
public final class Calibrations {

    /**
     * Calibration specific to the Intake subsystem.
     */
    public static final class Intake {
        public static final double P = 6e-5;
        public static final double D = 0.0;
        public static final double FF = 0.000015;
        public static final double extendedTargetRpm = 0.9 * 11000.0;
        public static final double retractedTargetRpm = 0.0;
    }

    /**
     * Calibration specific to the Tower subsystem.
     */
    public static final class Tower {
        public static final double P = 6e-5;
        public static final double D = 0.0;
        public static final double FF = 0.000015;
        public static final double loadTargetRpm = 0.5 * 11000.0;
        public static final double unloadTargetRpm = 0.5 * 11000.0;
    }


    /** 
     * The average wheel diameter (best: calibrate with encoders, ok: use tape measure).
     *
     * <p>To measure the average wheel diameter using the encoders requires a straight wall and chunk of carpet. The
     * procedure goes as follows:
     *
     * <p>1. Place the carpet up against the wall.
     *
     * <p>2. Place a side of the robot flush against a straight wall and on the carpet.
     *
     * <p>3. Mark the front of the robot on the wall.
     *
     * <p>4. Zero the encoders.
     *
     * <p>5. Manually push the robot forward (keep the robot in contact with the wall the whole time) until the encoders
     * indicate exactly 10 complete rotations.
     *
     * <p>6. Mark the front of the robot on the wall.
     *
     * <p>7. Measure the distance between the 2 marks on the wall.
     *
     * <p>8. The average wheel diameter is the measured distance divided by 10*pi.
     *
     * <p>This should be done before each event minimum
     */
    public static final double WHEEL_DIAMETER_INCH = 4.0; // TODO: Calibrate with encoders


    /**
     * Distance between right and left wheels.
     *
     * <p>To measure the track width, use the following procedure:
     * 
     * <p>1. Use a tape measure and record the distance between each pair of left and right wheels.
     * 
     * <p>2. The track width is the average of the two measurements from the first step.
     *
     * <p>Any changes to the track width from step 2 likely indicates there is mechanical damage to the module. Also, if the
     * difference between the left/right wheel pairs is large, this indicates an "out-of-square" frame.
     *
     * <p>This should be done before each event minimum
     */
    public static final double TRACK_WIDTH_INCH = 19.5;


    /**
     * Distance between front and rear wheels.
     *
     * <p>To measure the wheel base, use the following procedure:
     * 
     * <p>1. Use a tape measure and record the distance between each pair of front and rear wheels.
     * 
     * <p>2. The wheel base is the average of the two measurements from the first step.
     *
     * <p>Any changes to the wheel base from step 2 likely indicates there is mechanical damage to the module. Also, if the
     * difference between the front/rear wheel pairs is large, this indicates an "out-of-square" frame.
     *
     * <p>This should be done before each event minimum
     */
    public static final double WHEEL_BASE_INCH = 19.5;


    /**
     * The absolute encoder angle for homing the wheel (best: calibrate with encoders, ok: use a straight edge to align the
     * wheels).
     *
     * <p>To measure the absolute encoder home angle using the encoders requires a straight wall and chunk of carpet. The
     * procedure goes as follows:
     * 
     * <p>1. Place the carpet up against the wall.
     *
     * <p>2. Place a side of the robot flush against a straight wall and on the carpet.
     *
     * <p>3. Start recording the absolute encoder values.
     *
     * <p>4. Manually push the robot forward (keep the robot in contact with the wall the whole time) for ~10 feet.
     *
     * <p>5. Stop recording the absolute encoder values.
     *
     * <p>6. The homing angle is the average of the recorded values. Depending on the initial alignment of the wheels, the
     * beginning portion of the recorded values should be removed from the averaging.
     *
     * <p>The ordering of the values below are : front left, front right, rear left, rear right.
     *
     * <p>This should be done before each event minimum
     */
    public static final double[] ZEROS_RAD = {1.27, 0.91, 0.27, 5.82};


    /**
     * The maximum turning controller error allowed to be considered on-target.
     *
     * <p>This value should be tuned such that it is tight enough to properly home the wheels, but not too tight where the
     * homing routine oscillates and never reaches the goal.
     *
     * <p>This should be tuned before each event minimum
     */
    public static final double MAX_TURN_ERROR_DEG = 2.0; // TODO: How to monitor


    /**
     * The maximum angular velocity used for constraining trajectory profile of the turning controller.
     *
     * <p>Measuring the maximum angular velocity of the turning motor requires a chunk of carpet.The procedure goes as
     * follows:
     * 
     * <p>1. Place robot on the carpet.
     *
     * <p>2. Start recording the relative encoder velocity values.
     *
     * <p>3. Send full positive voltage to the turning motor.
     *
     * <p>4. Stop recording.
     *
     * <p>5. The maximum angular velocity is the average of the recorded values. Depending on the acceleration, the
     * beginning portion of the recorded values should be removed from the averaging.
     * 
     * <p>6. Repeat steps 2-5 using a full negative voltage.
     * 
     * <p>7. Take 90% of the minimum from step 5 (positive/negative voltage commands) and use this as the maximum angular
     * velocity. 
     *
     * <p>The ordering of the values below are : front left, front right, rear left, rear right.
     *
     * <p>This should be measured before each event minimum
     */
    public static final double[] MAX_TURN_VELOCITY_RPS = 
        {10 * Math.PI, 10 * Math.PI, 10 * Math.PI, 10 * Math.PI}; // TODO: Part of turning FF calibration


    /**
     * The maximum angular acceleration used for constraining trajectory profile of the turning controller.
     *
     * <p>Measuring the maximum angular acceleration of the turning motor requires a chunk of carpet. The procedure goes as
     * follows:
     * 
     * <p>1. Place robot on the carpet.
     *
     * <p>2. Start recording the relative encoder velocity values.
     *
     * <p>3. Send full positive voltage to the turning motor.
     *
     * <p>4. Stop recording.
     *
     * <p>5. The accelerations are the delta velocity measurements. Only keep the acceleration values starting with the
     * initial max voltage and ending with the motor reaching cruise velocity. Take the average of these remaining data.
     * 
     * <p>6. Since the acceleration data are noisy and short, repeat steps 3-5 several times. The maximum angular
     * acceleration is the average of all the step 5's. 
     * 
     * <p>7. Repeat steps 2-6 using a full negative voltage.
     * 
     * <p>8. Take 90% of the minimum from step 6 (positive/negative voltage commands) and use this as the maximum angular
     * acceleration. 
     *
     * <p>The ordering of the values below are : front left, front right, rear left, rear right.
     *
     * <p>This should be tuned before each event minimum
     */
    public static final double[] MAX_TURN_ACCELERATION_RPSS = 
        {10 * Math.PI, 10 * Math.PI, 10 * Math.PI, 10 * Math.PI}; // TODO: Part of turning FF calibration


    /**
     * The P-gain of the PID turning controller.
     *
     * <p>The same gain is applied across all of the swerve modules and modes.
     *
     * <p>This should be tuned before each event minimum
     */
    public static final double TURN_P_GAIN = 4.0; // TODO: Do after FF is implemented


    /**
     * The D-gain of the PID turning controller.
     *
     * <p>The same gain is applied across all of the swerve modules and modes.
     *
     * <p>This should be tuned before each event minimum
     */
    public static final double TURN_D_GAIN = 0.0; // TODO: Do after FF is implemented


    /**
     * The static gain of the feedforward turning controller.
     *
     * <p>Use the WPILib system identification tool to collect data and fit it to the permanent-magnet DC motor model.
     *
     * <p>The ordering of the values below are : front left, front right, rear left, rear right.
     *
     *<p>This should be tuned before each event minimum
     *
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html">WPILib System Identification</a>
     */
    public static final double[] TURN_FF_KS_GAIN = {0.0, 0.0, 0.0, 0.0}; // TODO: Part of turning FF calibration


    /**
     * The velocity gain of the feedforward turning controller.
     *
     * <p>Use the WPILib system identification tool to collect data and fit it to the permanent-magnet DC motor model.
     *
     * <p>The ordering of the values below are : front left, front right, rear left, rear right.
     *
     * <p>This should be tuned before each event minimum
     *
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html">WPILib System Identification</a>
     */
    public static final double[] TURN_FF_KV_GAIN = {0.0, 0.0, 0.0, 0.0}; // TODO: Part of turning FF calibration


    /**
     * The acceleration gain of the feedforward turning controller.
     *
     * <p>Use the WPILib system identification tool to collect data and fit it to the permanent-magnet DC motor model.
     *
     * <p>The ordering of the values below are : front left, front right, rear left, rear right.
     *
     * <p>This should be tuned before each event minimum
     *
     *  @see <a href="https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html">WPILib System Identification</a>
     */
    public static final double[] TURN_FF_KA_GAIN = {0.0, 0.0, 0.0, 0.0}; // TODO: Part of turning FF calibration


    /**
     * The P-gain of the PID driving controller.
     *
     * <p>The same gain is applied across all of the swerve modules and modes.
     *
     * <p>This should be tuned before each event minimum
     */
    public static final double DRIVE_P_GAIN = 3.0; // TODO: Do after FF is implemented


    /**
     * The D-gain of the PID driving controller.
     *
     * <p>The same gain is applied across all of the swerve modules and modes.
     *
     * <p>This should be tuned before each event minimum
     */
    public static final double DRIVE_D_GAIN = 0.0; // TODO: Do after FF is implemented


    /**
     * The static gain of the feedforward driving controller.
     *
     * <p>Use the WPILib system identification tool to collect data and fit it to the permanent-magnet DC motor model.
     *
     * <p>The ordering of the values below are : front left, front right, rear left, rear right.
     *
     * <p>This should be tuned before each event minimum
     *
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html">WPILib System Identification</a>
     */
    public static final double[] DRIVE_FF_KS_GAIN = {0.0, 0.0, 0.0, 0.0}; // TODO: Write command and calibrate


    /**
     * The velocity gain of the feedforward driving controller.
     *
     * <p>Use the WPILib system identification tool to collect data and fit it to the permanent-magnet DC motor model.
     *
     * <p>The ordering of the values below are : front left, front right, rear left, rear right.
     *
     * <p>This should be tuned before each event minimum
     *
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html">WPILib System Identification</a>
     */
    public static final double[] DRIVE_FF_KV_GAIN = {0.0, 0.0, 0.0, 0.0}; // TODO: Write command and calibrate


    /**
     * The acceleration gain of the feedforward driving controller.
     *
     * <p>Use the WPILib system identification tool to collect data and fit it to the permanent-magnet DC motor model.
     *
     * <p>The ordering of the values below are : front left, front right, rear left, rear right.
     *
     * <p>This should be tuned before each event minimum
     *
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html">WPILib System Identification</a>
     */
    public static final double[] DRIVE_FF_KA_GAIN = {0.0, 0.0, 0.0, 0.0}; // TODO: Write command and calibrate


    /**
     * The maximum drive velocity used to constrain the swerve drive signal.
     * 
     * <p>This is used as a driving speed limit. As the driver gets better this value can be increased towards maximum.
     *
     * <p>This should be set before each event minimum
     */
    public static final double MAX_DRIVE_VELOCITY_MPS = 2.0; // TODO: Write command and calibrate

}
