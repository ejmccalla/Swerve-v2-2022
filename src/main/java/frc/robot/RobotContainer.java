package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

/**
 * The RobotContainer class contains all of the subsystems, commands, and defines how the users
 * interface with the robot.
 */
public class RobotContainer {

    private final Joystick m_leftJoystick;
    private final Joystick m_rightJoystick;
    private final JoystickButton m_rightJoystickButton;
    public final Drivetrain m_drivetrain;


    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    //-------------------------------------------------------------------------------------------//
    /*                            CONSTRUCTOR AND PERIODIC METHODS                               */
    //-------------------------------------------------------------------------------------------//


    /**
     * Constructor for the robot container.
     */
    public RobotContainer() {
        m_leftJoystick = new Joystick(Constants.DriverStation.LEFT_JOYSTICK);
        m_rightJoystick = new Joystick(Constants.DriverStation.RIGHT_JOYSTICK);
        m_rightJoystickButton = new JoystickButton(m_rightJoystick, 1);
        m_drivetrain = new Drivetrain();

        m_drivetrain.setDefaultCommand(new Drive(m_leftJoystick, m_rightJoystick, m_drivetrain));

        // TODO: This setup will only run robot-oriented while a button is held. This may not be
        // ideal is the drive ALWAYS wants to run in robot-oriented (maybe the IMU drift gets so
        // far off that field-oriented isn't useful)
        m_rightJoystickButton.whenPressed(
            new InstantCommand(() -> m_drivetrain.setIsFieldOriented(false), m_drivetrain)
        );
        m_rightJoystickButton.whenReleased(
            new InstantCommand(() -> m_drivetrain.setIsFieldOriented(true), m_drivetrain)
        );

    }

}
