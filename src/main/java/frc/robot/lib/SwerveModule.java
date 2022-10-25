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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Calibrations;
import frc.robot.Constants;

/**
 * Implements the turning and driving controllers of a swerve module.
 *
 * <p>The turning contoller is a combination of feedback and feedforward control. The feedback
 * control is achieved using a profiled PID controller (trapezoidal profile and constraints)
 * with either the relative or absolute encoders for feedback. The absolute encoder is used to
 * "home" modules and the relative encoder is used after the "homing" process has completed. This
 * strategy was choosen to allow the most flexibility during on-field setup since the wheels will
 * be aligned properly (facing the correct direction and not reversed in drive direction). The
 * feedforward control is achieved by using a simple permanent-magnet DC motor model which zeros
 * the acceleration coefficient (due to the trapezoid profile implementation).
 *
 * <p>The driving controller is also a combination of feedback and feedforward control. The
 * feedback control is achieved using a velocity controller and the REV NEO onboard relative
 * encoder. The feedforward control is achieved by using a simple permanent-magnet DC motor model
 * which zeros the acceleration coefficient (due to the trapezoid profile implementation).
 */
public class SwerveModule {
    
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
    private SwerveModuleState m_setState;
    private double m_turnFeedForwardOutput;
    private double m_driveFeedForwardOutput;
    private double m_turnPidOutput;
    private double m_drivePidOutput;


    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Get the turning absolute encoder position and convert it to an angle.
     *
     * @return the angle in radians.
     */
    public double getTurnAbsEncAngleRad() {
        return m_turnAbsEnc.getAbsolutePosition() * 2.0 * Math.PI;
    }

    /**
     * Get the turning relative encoder distance which already has the raw encoder counts to
     * radians conversion baked in.
     *
     * @return the angle in radians. 
     */
    public double getTurnRelEncAngleRad() {
        return m_turnRelEnc.getDistance();
    }

    /**
     * Get the drive encoder velocity which already has the encoder RPM native units to
     * meters-per-second conversion baked in.
     *
     * @return the velocity in meters-per-second.
     */
    public double getDriveEncVelMps() {
        return m_driveEnc.getVelocity();
    }    

    /**
     * Get the current state of the swerve module as a drive speed in meters-per-second and an
     * angle represented as a 2D rotation.
     *
     * @return the current state of the swerve module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveEncVelMps(), 
                                     new Rotation2d(getTurnRelEncAngleRad()));
    }

    public double getPosSetpointRad() {
        return m_turnController.getSetpoint().position;
    }

    public double getVelSetpointRps() {
        return m_turnController.getSetpoint().velocity;
    }

    public double getPosErrorRad() {
        return m_turnController.getPositionError();
    }

    public double getVelErrorRps() {
        return m_turnController.getVelocityError();
    }

    /** Use the absolute encoder to home the swerve module.
     *
     * <p>The turning output voltage is calculated using a combination of the feedback and
     * feedforward controllers. The absolute encoder is used for feedback.
    */
    public void setHomingState() {
        m_turnPidOutput = 
            m_turnController.calculate(getTurnAbsEncAngleRad(), m_homeRad);
        m_turnFeedForwardOutput = 
            m_turnFeedForward.calculate(m_turnController.getSetpoint().velocity);

        m_turnMotor.setVoltage(m_turnPidOutput + m_turnFeedForwardOutput);
        m_driveMotor.setVoltage(0.0);
    }

    /**
     * Set the desired state of the swerve module.
     *
     * <p>The input swerve state is a speed in meters per second and an angle represented as a 2D
     * rotation. The desired angle is first optimized to keep the wheel from turning more than 90
     * degrees. This is done by allowing the wheel to reverse the current drive direction. Next, the
     * turning and driving output voltages are calculated using a combination of the feedback and
     * feedforward controllers. Finally, the turning and driving output voltages are sent out to
     * the motors.
     *
     * @param state the desired state of the swerve module
     */
    public void setState(SwerveModuleState state) {
        m_setState = SwerveModuleState.optimize(state, new Rotation2d(getTurnRelEncAngleRad()));

        m_turnPidOutput = 
            m_turnController.calculate(getTurnRelEncAngleRad(), m_setState.angle.getRadians());
        m_turnFeedForwardOutput = 
            m_turnFeedForward.calculate(m_turnController.getSetpoint().velocity);

        m_drivePidOutput =
            m_driveController.calculate(getDriveEncVelMps(), 
                                        m_setState.speedMetersPerSecond);
        m_driveFeedForwardOutput =
            m_driveFeedForward.calculate(state.speedMetersPerSecond);

        m_turnMotor.setVoltage(m_turnPidOutput + m_turnFeedForwardOutput);
        m_driveMotor.setVoltage(m_drivePidOutput + m_driveFeedForwardOutput);
    }

    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    //-------------------------------------------------------------------------------------------//
    /*                            CONSTRUCTOR AND PERIODIC METHODS                               */
    //-------------------------------------------------------------------------------------------//

    /**
     * Constructor for a SparkMax swerve drive module using the RoboRIO to control both the
     * steering and driving.
     *
     * @param home the zero of the module in radians
     * @param turnMotorId the turning motor controller CAN ID
     * @param driveMotorId the driving motor controller CAN ID
     * @param pwmChannel the RoboRIO DIO channel number of the encoder PWM signal
     * @param quadChannelA the RoboRIO DIO channel number of the encoder quadrature A signal
     * @param quadChannelB the RoboRIO DIO channel number of the encoder quadrature B signal
     * @param invertDrive a flag to indicate the drive motor is inverted
     */
    public SwerveModule(double home, int turnMotorId, int driveMotorId, int pwmChannel,
                        int quadChannelA, int quadChannelB, boolean invertDrive, double loopTime) {
        m_homeRad = home;

        m_turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);

        m_turnAbsEnc = new DutyCycleEncoder(pwmChannel);
        m_turnRelEnc = new Encoder(quadChannelA, quadChannelB);
        m_driveEnc = m_driveMotor.getEncoder();

        m_profiledPidConstraints =
            new TrapezoidProfile.Constraints(Calibrations.MAX_TURN_VELOCITY_RPS,
                                             Calibrations.MAX_TURN_ACCELERATION_RPSS);
        m_turnController =
            new ProfiledPIDController(Calibrations.TURN_P_GAIN, 0.0, Calibrations.TURN_D_GAIN, 
                                      m_profiledPidConstraints, loopTime);
        m_driveController =
            new PIDController(Calibrations.DRIVE_P_GAIN, 0.0, Calibrations.DRIVE_D_GAIN, loopTime);

        m_turnFeedForward =
            new SimpleMotorFeedforward(Calibrations.DRIVE_FF_KS_GAIN,
                                       Calibrations.DRIVE_FF_KV_GAIN);
        m_driveFeedForward =
            new SimpleMotorFeedforward(Calibrations.TURN_FF_KS_GAIN,
                                       Calibrations.TURN_FF_KV_GAIN);
        
        double baseConversion =
            (Units.inchesToMeters(Calibrations.WHEEL_DIAMETER_INCH) * Math.PI
            / Constants.Drivetrain.DRIVE_GEAR_RATIO);
        m_driveEnc.setPositionConversionFactor(baseConversion);
        m_driveEnc.setVelocityConversionFactor(baseConversion / 60.0);
        m_turnRelEnc.setDistancePerPulse(2.0 * Math.PI / 4096.0);

        //m_turnController.enableContinuousInput(-Math.PI, Math.PI);
        m_turnController.disableContinuousInput();
        m_turnController.setTolerance(Units.degreesToRadians(Calibrations.MAX_TURN_ERROR_DEG));

        m_turnMotor.restoreFactoryDefaults();
        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setInverted(invertDrive);

        m_driveEnc.setPosition(0);
        m_turnRelEnc.reset();

    }

}
