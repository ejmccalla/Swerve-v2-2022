package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.lib.SwerveModule;

/**
 * Implementation of a swerve drivetrain using REV/SparkMax.
 */
public class Drivetrain extends SubsystemBase {

    private enum StateType {
        Homing { 
            @Override
            public String toString() {
                return "Homing";
            }
        },
        Teleop {
            @Override
            public String toString() {
                return "Teleop";
            }
        },
        Following {
            @Override
            public String toString() {
                return "Following";
            }
        },
    }

    private final SwerveModule[] m_modules;
    private final SwerveDriveKinematics m_kinematics;
    private final ADIS16470_IMU m_imu;
    private final SwerveDriveOdometry m_odometry;
    private SwerveModuleState[] m_swerveModuleStates;
    private ChassisSpeeds m_chassisSpeeds;
    private StateType m_currentState;


    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Method to drive the robot using joystick info.
     *
     * @param speedX Speed of the robot in the x direction (forward).
     * @param speedY Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double speedX, double speedY, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            m_chassisSpeeds = 
                ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot, getImuRotation());
        } else {
            m_chassisSpeeds = new ChassisSpeeds(speedX, speedY, rot);
        }
        m_swerveModuleStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        setModuleStates(m_swerveModuleStates);
    }

    /**
     * Method to output calibration telemetry to the smart dashboard.
     */
    public void outputCalibrationTelemetry() {
        //new SwerveModule[]{frontLeft, frontRight, rearLeft, rearRight};
        String[] modules = {"FL", "FR", "RL", "RR"}; 
        for (int i = 0; i < m_modules.length; i++) {
            SmartDashboard.putNumber(modules[i] + " Absolute Encoder (rad)", 
                m_modules[i].getTurnAbsEncAngleRad());
            SmartDashboard.putNumber(modules[i] + " Position Setpoint (rad)", 
                m_modules[i].getPosSetpointRad());
            SmartDashboard.putNumber(modules[i] + " Velocity Setpoint (rad/s)", 
                m_modules[i].getVelSetpointRps());
            SmartDashboard.putNumber(modules[i] + " Position Error (rad)", 
                m_modules[i].getPosErrorRad());
            SmartDashboard.putNumber(modules[i] + " Velocity Error (rad/s)", 
                m_modules[i].getVelErrorRps());
        }
    }


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /** Returns the IMU yaw angle as a Rotation2d object. */
    private Rotation2d getImuRotation() {
        return Rotation2d.fromDegrees(m_imu.getAngle());
    }

    /** Updates the field relative position of the robot. */
    private void updateOdometry() {
        m_odometry.update(
            getImuRotation(),
            m_modules[0].getState(),
            m_modules[1].getState(),
            m_modules[2].getState(),
            m_modules[3].getState());
    }

    /** Set the swerve module states. */
    private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, 
                                                    Calibrations.MAX_DRIVE_VELOCITY_MPS);
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setState(swerveModuleStates[i]);
        }
    }


    //-------------------------------------------------------------------------------------------//
    /*                            CONSTRUCTOR AND PERIODIC METHODS                               */
    //-------------------------------------------------------------------------------------------//


    /** Constructor for the drivetrain. */
    public Drivetrain() {
        SwerveModule frontLeft =
            new SwerveModule(Calibrations.FRONT_LEFT_ZERO_RAD,
                             Constants.Drivetrain.FRONT_LEFT_TURN_ID,
                             Constants.Drivetrain.FRONT_LEFT_DRIVE_ID,
                             Constants.Drivetrain.FRONT_LEFT_PWM_DIO_CHANNEL,
                             Constants.Drivetrain.FRONT_LEFT_QUAD_A_DIO_CHANNEL,
                             Constants.Drivetrain.FRONT_LEFT_QUAD_B_DIO_CHANNEL,
                             false,
                             0.02);

        SwerveModule frontRight =
            new SwerveModule(Calibrations.FRONT_RIGHT_ZERO_RAD,
                             Constants.Drivetrain.FRONT_RIGHT_TURN_ID,
                             Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID,
                             Constants.Drivetrain.FRONT_RIGHT_PWM_DIO_CHANNEL,
                             Constants.Drivetrain.FRONT_RIGHT_QUAD_A_DIO_CHANNEL,
                             Constants.Drivetrain.FRONT_RIGHT_QUAD_B_DIO_CHANNEL,
                             false,
                             0.02);

        SwerveModule rearLeft =
            new SwerveModule(Calibrations.REAR_LEFT_ZERO_RAD,
                             Constants.Drivetrain.REAR_LEFT_TURN_ID,
                             Constants.Drivetrain.REAR_LEFT_DRIVE_ID,
                             Constants.Drivetrain.REAR_LEFT_PWM_DIO_CHANNEL,
                             Constants.Drivetrain.REAR_LEFT_QUAD_A_DIO_CHANNEL,
                             Constants.Drivetrain.REAR_LEFT_QUAD_B_DIO_CHANNEL,
                             false,
                             0.02);

        SwerveModule rearRight =
            new SwerveModule(Calibrations.REAR_RIGHT_ZERO_RAD,
                             Constants.Drivetrain.REAR_RIGHT_TURN_ID,
                             Constants.Drivetrain.REAR_RIGHT_DRIVE_ID,
                             Constants.Drivetrain.REAR_RIGHT_PWM_DIO_CHANNEL,
                             Constants.Drivetrain.REAR_RIGHT_QUAD_A_DIO_CHANNEL,
                             Constants.Drivetrain.REAR_RIGHT_QUAD_B_DIO_CHANNEL,
                             false,
                             0.02);
        m_modules = new SwerveModule[]{frontLeft, frontRight, rearLeft, rearRight};
        
        m_kinematics = new SwerveDriveKinematics(Constants.Drivetrain.FRONT_LEFT_LOCATION,
                                                 Constants.Drivetrain.FRONT_RIGHT_LOCATION, 
                                                 Constants.Drivetrain.REAR_LEFT_LOCATION,
                                                 Constants.Drivetrain.REAR_RIGHT_LOCATION);

        m_imu = new ADIS16470_IMU();
        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
        m_currentState = StateType.Homing;
    }

    /** This method is called periodically. */
    @Override 
    public void periodic() {
        updateOdometry();
        switch (m_currentState) {
            case Homing:
                for (int i = 0; i < m_modules.length; i++) {
                    m_modules[i].setHomingState();
                }
                break;

            default:
                for (int i = 0; i < m_modules.length; i++) {
                    m_modules[i].setState(new SwerveModuleState(0.0, 
                        new Rotation2d(m_modules[i].getTurnRelEncAngleRad())));
                }
        }
    }


}
