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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Calibrations;
import frc.robot.Constants;

/**
 * Implementation of the turning and driving control of a swerve module.
 */
public class SwerveModule {
    
    private final double m_home;
    private final CANSparkMax m_turnMotor;
    private final CANSparkMax m_driveMotor;
    private final DutyCycleEncoder m_turnAbsoluteEncoder;
    private final Encoder m_turnRelativeEncoder;
    private final RelativeEncoder m_driveEncoder;
    private final ProfiledPIDController m_turnController;
    private final PIDController m_driveController;
    private final SimpleMotorFeedforward m_turnFeedForward;
    private final SimpleMotorFeedforward m_driveFeedForward;
    private SwerveModuleState m_setState;
    private double m_turnFeedForwardOutput;
    private double m_driveFeedForwardOutput;
    private double m_turnPidOutput;
    private double m_drivePidOutput;

    /**
     * Set the desired state of the swerve module (drive in meters per second and heading in
     * radians).
     *
     * @param state the desired state of the swerve module
     */
    public void setState(SwerveModuleState state) {
        m_setState = SwerveModuleState.optimize(state, new Rotation2d(m_turnRelativeEncoder.get()));

        m_turnPidOutput = 
            m_turnController.calculate(m_turnRelativeEncoder.get(), m_setState.angle.getRadians());
        m_turnFeedForwardOutput = 
            m_turnFeedForward.calculate(m_turnController.getSetpoint().velocity);

        m_drivePidOutput =
            m_driveController.calculate(m_driveEncoder.getVelocity(), 
                                        m_setState.speedMetersPerSecond);
        m_driveFeedForwardOutput =
            m_driveFeedForward.calculate(state.speedMetersPerSecond);

        m_turnMotor.setVoltage(m_turnPidOutput + m_turnFeedForwardOutput);
        m_driveMotor.setVoltage(m_drivePidOutput + m_driveFeedForwardOutput);
    }

    /** Use the absolute encoder to home the turning. */
    public void setHomingState() {
        m_turnPidOutput = 
            m_turnController.calculate(m_turnAbsoluteEncoder.get(), m_home);
        m_turnFeedForwardOutput = 
            m_turnFeedForward.calculate(m_turnController.getSetpoint().velocity);

        m_turnMotor.setVoltage(m_turnPidOutput + m_turnFeedForwardOutput);
        m_driveMotor.setVoltage(0.0);
    }


    public double getHome() {
        return m_home;
    }

    public double getAbsolutEncoderAngle() {
        return m_turnAbsoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI;
    }

    public double getTurnRelativeEncoderAngle() {
        return m_turnRelativeEncoder.get();
    }

    public double getDriveEncoderPosition() {
        return m_driveEncoder.getPosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), 
                                     new Rotation2d(m_turnRelativeEncoder.get()));
    }

    public void setEncoderHomeOffset() {
        m_turnRelativeEncoder.reset();  //setPosition(getAbsolutEncoderAngle() - m_home);
    }

    /**
     * Constructor for a SparkMax swerve drive module using the RoboRIO to control both the
     * steering and driving.
     *
     * @param label a informative description of the module, like Front-Left
     * @param home the zero of the module
     * @param turnMotorId the turning motor controller CAN ID
     * @param driveMotorId the driving motor controller CAN ID
     * @param pwmChannel the RoboRIO DIO channel number of the encoder PWM signal
     * @param quadChannelA the RoboRIO DIO channel number of the encoder quadrature A signal
     * @param quadChannelB the RoboRIO DIO channel number of the encoder quadrature B signal
     * @param invertDrive a flag to indicate the drive motor is inverted
     */
    public SwerveModule(String label, double home, int turnMotorId, int driveMotorId,
                        int pwmChannel, int quadChannelA, int quadChannelB, boolean invertDrive,
                        double loopTime) {
        m_home = home;

        m_turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);

        m_turnAbsoluteEncoder = new DutyCycleEncoder(pwmChannel);
        m_turnRelativeEncoder = new Encoder(quadChannelA, quadChannelB);
        m_driveEncoder = m_driveMotor.getEncoder();

        var constraints = 
            new TrapezoidProfile.Constraints(Calibrations.MAX_TURN_VELOCITY_RPS,
                                             Calibrations.MAX_TURN_ACCELERATION_RPSS);
        m_turnController =
            new ProfiledPIDController(Calibrations.TURN_P_GAIN, 0.0, Calibrations.TURN_D_GAIN, 
                                      constraints, loopTime);
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
        m_driveEncoder.setPositionConversionFactor(baseConversion);
        m_driveEncoder.setVelocityConversionFactor(baseConversion / 60.0);
        m_turnRelativeEncoder.setDistancePerPulse(2.0 * Math.PI / 4096.0);

        m_turnController.enableContinuousInput(-Math.PI, Math.PI);
        m_turnController.setTolerance(Units.degreesToRadians(Calibrations.MAX_TURN_ERROR_DEG));

        m_turnMotor.restoreFactoryDefaults();
        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setInverted(invertDrive);

        m_driveEncoder.setPosition(0);
        m_turnRelativeEncoder.reset();

        ShuffleboardLayout layout = Shuffleboard.getTab("Drivetrain").getLayout(label, "list");
        layout.addNumber("Home (Rad)", this::getHome);
        layout.addNumber("Absolute Turn Encoder (Rad)", this::getAbsolutEncoderAngle);
        layout.addNumber("Relative Turn Encoder (Rad)", this::getTurnRelativeEncoderAngle);
        layout.addNumber("Drive Encoder (M)", this::getDriveEncoderPosition);

    }

}
