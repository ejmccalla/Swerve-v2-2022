package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
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
                return "Drive";
            }
        },
        PathFollowing {
            @Override
            public String toString() {
                return "Path Following";
            }
        },
    }

    private final SwerveModule[] m_modules;
    private final SwerveDriveKinematics m_kinematics;
    private final ADIS16470_IMU m_imu;
    private final SwerveDriveOdometry m_odometry;
    private SwerveModuleState[] m_currentStates;
    private SwerveModuleState[] m_desiredStates;
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
        m_desiredStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    }


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /** Returns the IMU yaw angle as a Rotation2d object. */
    private Rotation2d getImuRotation() {
        return Rotation2d.fromDegrees(m_imu.getAngle());
    }

    /** Set the swerve module states. */
    private void setModulesDesiredState() {
        SwerveDriveKinematics.desaturateWheelSpeeds(m_desiredStates, 
                                                    Calibrations.MAX_DRIVE_VELOCITY_MPS);
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setState(m_desiredStates[i]);
        }
    }


    //-------------------------------------------------------------------------------------------//
    /*                            CONSTRUCTOR AND PERIODIC METHODS                               */
    //-------------------------------------------------------------------------------------------//


    /** Constructor for the drivetrain. */
    public Drivetrain() {
        m_modules = new SwerveModule[Constants.Drivetrain.numModules];
        m_desiredStates = new SwerveModuleState[Constants.Drivetrain.numModules];
        m_currentStates = new SwerveModuleState[Constants.Drivetrain.numModules];
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i] = new SwerveModule(Constants.Drivetrain.MODULE_LABELS[i],
                                            Calibrations.ZEROS_RAD[i],
                                            Constants.Drivetrain.TURN_IDS[i],
                                            Constants.Drivetrain.DRIVE_IDS[i],
                                            Constants.Drivetrain.PWM_DIO_CHANNELS[i],
                                            Constants.Drivetrain.QUAD_A_DIO_CHANNELS[i],
                                            Constants.Drivetrain.QUAD_B_DIO_CHANNELS[i],
                                            0.02);
            m_desiredStates[i] = new SwerveModuleState(0.0, new Rotation2d());
            m_currentStates[i] = m_modules[i].getState();
        }
        m_kinematics = new SwerveDriveKinematics(Constants.Drivetrain.MODULE_LOCATIONS);
        m_imu = new ADIS16470_IMU();
        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
        m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        m_currentState = StateType.Homing;

    }

    /** This method is called periodically. */
    @Override 
    public void periodic() {

        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].logTelemetry();
            m_currentStates[i] = m_modules[i].getState();
        }
        m_odometry.update(getImuRotation(), m_currentStates);

        switch (m_currentState) {
            case Homing:
                for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
                    m_modules[i].homeModule();
                }
                break;

            default:
                for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
                    m_desiredStates[i] = m_modules[i].getState();
                    m_desiredStates[i].speedMetersPerSecond = 0.0;
                }
                setModulesDesiredState();
        }
    }

}
