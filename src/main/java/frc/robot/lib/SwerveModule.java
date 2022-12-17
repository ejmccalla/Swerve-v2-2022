package frc.robot.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Calibrations;
import frc.robot.Constants;

/**
 * Implements the turning and driving controllers of a swerve module.
 *
 * <p>The turning contoller is a combination of feedback and feedforward control. The feedback control is achieved using a
 * profiled PID controller (trapezoidal profile and constraints) with either the relative or absolute encoders for feedback.
 * The absolute encoder is used to "home" modules and the relative encoder is used after the "homing" process has completed.
 * This strategy was choosen to allow the most flexibility during on-field setup since the wheels will be aligned properly
 * (facing the correct direction and not reversed in drive direction). The feedforward control is achieved by using a simple
 * permanent-magnet DC motor model which zeros the acceleration coefficient (due to the trapezoid profile implementation).
 *
 * <p>The driving controller is also a combination of feedback and feedforward control. The feedback control is achieved
 * using a velocity controller and the REV NEO onboard relative encoder. The feedforward control is achieved by using a
 * simple permanent-magnet DC motor model which zeros the acceleration coefficient (due to the trapezoid profile
 * implementation).
 */
public class SwerveModule {

    private final String m_label;
    private final double m_homeRad;
    private final CANSparkMax m_turnMotor;
    private final CANSparkMax m_driveMotor;
    private final DutyCycleEncoder m_turnAbsEnc;
    private final Encoder m_turnRelEnc;
    private final RelativeEncoder m_driveEnc;
    private final ProfiledPIDController m_turnController;
    private final PIDController m_driveController;
    private final SimpleMotorFeedforward m_turnFeedForward;
    private final SimpleMotorFeedforward m_driveFeedForward;
    private final Constraints m_profiledPidConstraints;
    private final double m_baseConversion;
    private final DataLog m_log;
    private boolean m_isHomed;
    private SwerveModuleState m_setDesiredState;
    private double m_turnFeedForwardOutput;
    private double m_driveFeedForwardOutput;
    private double m_turnPidOutput;
    private double m_drivePidOutput;
    private DoubleLogEntry m_turnAbsEncLogEntry;
    private DoubleLogEntry m_turnRelEncLogEntry;
    private DoubleLogEntry m_turnRelEncVelocityLogEntry;
    private DoubleLogEntry m_driveEncLogEntry;
    private DoubleLogEntry m_turnPosSetpointLogEntry;
    private DoubleLogEntry m_turnVelSetpointLogEntry;
    private DoubleLogEntry m_turnPosErrorLogEntry;
    private DoubleLogEntry m_turnVelErrorLogEntry;
    private DoubleLogEntry m_turnFeedforwardOutLogEntry;
    private DoubleLogEntry m_turnPidOutLogEntry;
    private DoubleLogEntry m_turnApplidedVoltageLogEntry;
    private BooleanLogEntry m_isHomedLogEntry;


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                   PUBLIC METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Log the telemetry data to disk using the WPILib logger.
     */
    public void logTelemetry() {
        if (Constants.Drivetrain.ENABLE_LOGGING) {
            m_turnAbsEncLogEntry.append(getTurnAbsEncAngleRad());
            m_turnRelEncLogEntry.append(getTurnRelEncAngleRad());
            m_turnRelEncVelocityLogEntry.append(getTurnRelEncVelocityRps());
            m_driveEncLogEntry.append(getDriveEncVelMps());
            m_turnPosSetpointLogEntry.append(getTurnPosSetpointRad());
            m_turnVelSetpointLogEntry.append(getTurnVelSetpointRps());
            m_turnPosErrorLogEntry.append(getTurnPosErrorRad());
            m_turnVelErrorLogEntry.append(getTurnVelErrorRps());
            m_turnFeedforwardOutLogEntry.append(getTurnFeedforwardOutputV());
            m_turnPidOutLogEntry.append(getTurnPidOutputV());
            m_turnApplidedVoltageLogEntry.append(getTurnAppliedVoltage());
            m_isHomedLogEntry.append(getIsHomed());
        }
    }

    /**
     * Get the current state of the swerve module as a drive speed in meters-per-second and an angle represented as a
     * Rotation2D object.
     *
     * <p>If the module has been homed, the Rotation2D is based on the angle of the relative encoder. Otherwise, the
     * Rotation2D is based on the angle of the absolute encoder. 
     *
     * @return the current state of the swerve module
     */
    public SwerveModuleState getCurrentState() {
        if (m_isHomed) {
            return new SwerveModuleState(getDriveEncVelMps(), new Rotation2d(getTurnRelEncAngleRad()));
        } else {
            return new SwerveModuleState(0.0, new Rotation2d(getTurnAbsEncAngleRad()));
        }
    }

    /**
     * Reset the profiled PID turning controller which will zero out the integral term and update the setpoint to the
     * current angle of the absolute encoder.
     */
    public void resetTurningController() {
        m_turnController.reset(getTurnAbsEncAngleRad());
    }

    /**
     * Use the absolute encoder to home the swerve module.
     *
     * <p>The turning output voltage is calculated using a combination of the feedback and feedforward controllers. The
     * absolute encoder is used for feedback and the modelled model coeeficients are used for the feed-forward. If the
     * controller is already at the goal, the motor outputs are set to 0, the module state is updated to reflect the
     * completion of the homing, and the relative encoder is reset.
    */
    public void setHomedModuleState() {
        m_turnPidOutput = 
            m_turnController.calculate(getTurnAbsEncAngleRad(), m_homeRad);
        m_turnFeedForwardOutput = 
            m_turnFeedForward.calculate(m_turnController.getSetpoint().velocity);

        m_isHomed = false;
        if (m_turnController.atGoal()) {
            m_isHomed = true;
            m_turnRelEnc.reset();
            m_driveEnc.setPosition(0);
            m_turnPidOutput = 0.0;
            m_turnFeedForwardOutput = 0.0;
        }
        m_turnMotor.setVoltage(m_turnPidOutput + m_turnFeedForwardOutput);
        m_driveMotor.setVoltage(0.0);
    }

    /**
     * Get the homing state of the module.
     *
     * @return true if the module is homed, otherwise false
     */
    public boolean getIsHomed() {
        return m_isHomed;
    }

    /**
     * Set the desired state of the swerve module.
     *
     * <p>The input swerve state is a speed in meters-per-second and an angle represented as a Rotation2D object. The
     * desired angle is first optimized to keep the wheel from turning more than 90 degrees. This is done by allowing the
     * wheel to reverse the current drive direction. Next, the turning and driving output voltages are calculated using a
     * combination of feedback and feedforward controllers. Finally, the turning and driving output voltages are sent out to
     * the motors.
     *
     * @param state the desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState state) {
        m_setDesiredState =
            SwerveModuleState.optimize(state, new Rotation2d(getTurnRelEncAngleRad()));

        m_turnPidOutput = m_turnController.calculate(getTurnRelEncAngleRad(), m_setDesiredState.angle.getRadians());
        m_turnFeedForwardOutput = m_turnFeedForward.calculate(m_turnController.getSetpoint().velocity);

        m_drivePidOutput = m_driveController.calculate(getDriveEncVelMps(), m_setDesiredState.speedMetersPerSecond);
        m_driveFeedForwardOutput = m_driveFeedForward.calculate(state.speedMetersPerSecond);

        m_turnMotor.setVoltage(m_turnPidOutput + m_turnFeedForwardOutput);
        m_driveMotor.setVoltage(m_drivePidOutput + m_driveFeedForwardOutput);
    }

    /**
     * Sets the commanded voltage of the turn motor.
     *
     * @param voltage the voltage the motor is set to.
     */
    public void setTurnVoltage(double voltage) {
        m_turnMotor.setVoltage(voltage);
    }

    /**
     * This command will output 0 volts to the drive and turning motors.
     */
    public void setIdle() {
        m_turnMotor.setVoltage(0.0);
        m_driveMotor.setVoltage(0.0);
    }

    /**
     * Set the idle mode of the module motors.
     *
     * @param isBrakeDesired true sets brake mode, false sets coast mode
     */
    public void setModulesToBrakeMode(boolean isBrakeDesired) {
        if (isBrakeDesired) {
            m_turnMotor.setIdleMode(IdleMode.kBrake);
            m_driveMotor.setIdleMode(IdleMode.kBrake);    
        } else {
            m_turnMotor.setIdleMode(IdleMode.kCoast);
            m_driveMotor.setIdleMode(IdleMode.kCoast);    
        }
    }

    /**
     * Set the drive encoder postion to 0.
     */
    public void resetDriveEncoder() {
        m_driveEnc.setPosition(0);
    }

    /**
     * Get the drive encoder position in native units of rotations.
     *
     * @return the number of rotations
     */
    public double getDriveEncPosRot() {
        return m_driveEnc.getPosition();
    }



    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                  PRIVATE METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Get the turning absolute encoder position and convert it to an angle.
     *
     * @return the angle in radians
     */
    private double getTurnAbsEncAngleRad() {
        return m_turnAbsEnc.getAbsolutePosition() * 2.0 * Math.PI;
    }

    /**
     * Get the turning relative encoder distance which already has the raw encoder counts to radians conversion baked in.
     *
     * @return the angle in radians 
     */
    public double getTurnRelEncAngleRad() {
        return m_turnRelEnc.getDistance();
    }

    /**
     * Get the turning relative encoder velocity which already has the raw encoder counts to radians conversion baked in.
     *
     * @return the velocity in radians per second
     */
    private double getTurnRelEncVelocityRps() {
        return m_turnRelEnc.getRate();
    }

    /**
     * Get the drive encoder velocity which already has the encoder RPM native units to meters-per-second conversion baked
     * in.
     *
     * @return the velocity in meters-per-second
     */
    private double getDriveEncVelMps() {
        return m_driveEnc.getVelocity();
    }

    /**
     * Get the current turning controller position setpoint.
     *
     * @return the postion setpoint in radians
     */
    private double getTurnPosSetpointRad() {
        return m_turnController.getSetpoint().position;
    }

    /**
     * Get the current turning controller velocity setpoint.
     *
     * @return the velocity setpoint in radians-per-second
     */
    private double getTurnVelSetpointRps() {
        return m_turnController.getSetpoint().velocity;
    }

    /**
     * Get the current turning controller position error.
     *
     * @return the position error in radians
     */
    private double getTurnPosErrorRad() {
        return m_turnController.getPositionError();
    }

    /**
     * Get the current turning controller velocity error.
     *
     * @return the velocity error in radians-per-second
     */
    private double getTurnVelErrorRps() {
        return m_turnController.getVelocityError();
    }

    /**
     * Get the current turning controller feed-forward voltage output.
     *
     * @return the feed-forward output in volts
     */
    private double getTurnFeedforwardOutputV() {
        return m_turnFeedForwardOutput;
    }

    /**
     * Get the current turning controller PID voltage output.
     *
     * @return the PID output in volts
     */
    private double getTurnPidOutputV() {
        return m_turnPidOutput;
    }

    /**
     * The derived voltage from the duty cycle and the voltage fed into the motor controller.
     *
     * @return the derived voltage
     */
    private double getTurnAppliedVoltage() {
        return m_turnMotor.get() * m_turnMotor.getBusVoltage();
    }
    

    //--------------------------------------------------------------------------------------------------------------------//
    /*                                         CONSTRUCTOR AND PERIODIC METHODS                                           */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Constructor for a SparkMax swerve drive module using the RoboRIO to control both the steering and driving.
     *
     * @param label the module identifier
     * @param home the zero of the module in radians
     * @param turnMotorId the turning motor controller CAN ID
     * @param driveMotorId the driving motor controller CAN ID
     * @param pwmChannel the RoboRIO DIO channel number of the encoder PWM signal
     * @param quadChannelA the RoboRIO DIO channel number of the encoder quadrature A signal
     * @param quadChannelB the RoboRIO DIO channel number of the encoder quadrature B signal
     * @param maxTurnVelRps the turning motor maximum angular velocity in radians-per-second
     * @param maxTurnAccRpss the turning motor maximum angular acceleration in radians-per-second
     * @param turnFeedforwardKs the turning motor FF model static coefficient
     * @param turnFeedforwardKv the turning motor FF model velocity coefficient
     * @param turnFeedforwardKa the turning motor FF model acceleration coefficient
     * @param driveFeedforwardKs the driving motor FF model static coefficient
     * @param driveFeedforwardKv the driving motor FF model velocity coefficient
     * @param driveFeedforwardKa the driving motor FF model acceleration coefficient
     */
    public SwerveModule(String label, double home, int turnMotorId, int driveMotorId, int pwmChannel, int quadChannelA,
                        int quadChannelB, double maxTurnVelRps, double maxTurnAccRpss, double turnFeedforwardKs,
                        double turnFeedforwardKv, double turnFeedforwardKa, double driveFeedforwardKs,
                        double driveFeedforwardKv, double driveFeedforwardKa) {
        m_label = label;
        m_homeRad = home;

        m_turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        m_turnAbsEnc = new DutyCycleEncoder(pwmChannel);
        m_turnRelEnc = new Encoder(quadChannelA, quadChannelB);
        m_driveEnc = m_driveMotor.getEncoder();

        // TODO: Calibrate the velocity and acceleration constraints.
        m_profiledPidConstraints = new TrapezoidProfile.Constraints(maxTurnVelRps, maxTurnAccRpss);
        m_turnController =
            new ProfiledPIDController(Calibrations.TURN_P_GAIN, 0.0, Calibrations.TURN_D_GAIN, m_profiledPidConstraints);
        m_driveController = new PIDController(Calibrations.DRIVE_P_GAIN, 0.0, Calibrations.DRIVE_D_GAIN);

        // TODO: Calibrate the FF models.
        m_turnFeedForward = new SimpleMotorFeedforward(driveFeedforwardKs, driveFeedforwardKv, driveFeedforwardKa);
        m_driveFeedForward = new SimpleMotorFeedforward(turnFeedforwardKs, turnFeedforwardKv, turnFeedforwardKa);

        // TODO: Configure motors - current limit, CAN frame rates, limit switches, etc.
        m_turnMotor.restoreFactoryDefaults();
        m_driveMotor.restoreFactoryDefaults();

        m_baseConversion =
            (Units.inchesToMeters(Calibrations.WHEEL_DIAMETER_INCH) * Math.PI / Constants.Drivetrain.DRIVE_GEAR_RATIO);
        //m_driveEnc.setPositionConversionFactor(m_baseConversion);
        m_driveEnc.setVelocityConversionFactor(m_baseConversion / 60.0);
        m_turnRelEnc.setDistancePerPulse(2.0 * Math.PI / Constants.Drivetrain.turnEncPpr);
        m_turnController.setTolerance(Units.degreesToRadians(Calibrations.MAX_TURN_ERROR_DEG));
        m_turnController.enableContinuousInput(-Math.PI, Math.PI);

        m_driveEnc.setPosition(0);

        m_isHomed = false;
        m_setDesiredState = new SwerveModuleState(0.0, new Rotation2d(getTurnAbsEncAngleRad()));
        m_turnFeedForwardOutput = 0.0;
        m_driveFeedForwardOutput = 0.0;
        m_turnPidOutput = 0.0;
        m_drivePidOutput = 0.0;

        if (Constants.Drivetrain.ENABLE_LOGGING) {
            m_log = DataLogManager.getLog();
            m_turnAbsEncLogEntry = new DoubleLogEntry(m_log, m_label + " Turn Abs Enc (rad)");
            m_turnRelEncLogEntry = new DoubleLogEntry(m_log, m_label + " Turn Rel Enc (rad)");
            m_turnRelEncVelocityLogEntry = new DoubleLogEntry(m_log, m_label + " Turn Rel Enc Velocity (rad/s)");
            m_driveEncLogEntry = new DoubleLogEntry(m_log, m_label + " Drive Rel Enc (mps)");
            m_turnPosSetpointLogEntry = new DoubleLogEntry(m_log, m_label + " Turn Position Setpoint (rad)");
            m_turnVelSetpointLogEntry = new DoubleLogEntry(m_log, m_label + " Turn Velocity Setpoint (rad/s)");
            m_turnPosErrorLogEntry = new DoubleLogEntry(m_log, m_label + " Turn Position Error (rad)");
            m_turnVelErrorLogEntry = new DoubleLogEntry(m_log, m_label + " Turn Velocity Error (rad/s)");
            m_turnFeedforwardOutLogEntry = new DoubleLogEntry(m_log, m_label + " Turn Feed-forward Output (V)");
            m_turnPidOutLogEntry = new DoubleLogEntry(m_log, m_label + " Turn PID Output (V)");
            m_turnApplidedVoltageLogEntry = new DoubleLogEntry(m_log, m_label + " Turn Applied Voltage (V)");
            m_isHomedLogEntry = new BooleanLogEntry(m_log, m_label + " Is Homed");
        } else {
            m_log = null;
            m_turnAbsEncLogEntry = null;
            m_turnRelEncLogEntry = null;
            m_turnRelEncVelocityLogEntry = null;
            m_driveEncLogEntry = null;
            m_turnPosSetpointLogEntry = null;
            m_turnVelSetpointLogEntry = null;
            m_turnPosErrorLogEntry = null;
            m_turnVelErrorLogEntry = null;
            m_turnFeedforwardOutLogEntry = null;
            m_turnPidOutLogEntry = null;
            m_turnApplidedVoltageLogEntry = null;
            m_isHomedLogEntry = null;
        }
    }
}
