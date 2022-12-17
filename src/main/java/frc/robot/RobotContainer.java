package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.HomeSwerveModules;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;

/**
 * The RobotContainer class contains all of the subsystems, commands, and defines how the users interface with the robot.
 */
public class RobotContainer {

    private final Joystick m_leftJoystick;
    private final Joystick m_rightJoystick;
    private final JoystickButton m_leftJoystickButton;
    private final JoystickButton m_rightJoystickButton;
    public final Drivetrain m_drivetrain;
    public final Intake m_intake;
    public final Tower m_tower;


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                   PUBLIC METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                  PRIVATE METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                         CONSTRUCTOR AND PERIODIC METHODS                                           */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Constructor for the robot container.
     * 
     * <p>The default command for the drivetrain is the {@link frc.robot.commands.Drive} commamnd. The default behavior of
     * the drive command is to be field-oriented. The right joystick button is used to change this behavior. While this
     * button is pressed, the drive commands will be robot-oriented.
     */
    public RobotContainer() {
        m_leftJoystick = new Joystick(Constants.DriverStation.LEFT_JOYSTICK);
        m_rightJoystick = new Joystick(Constants.DriverStation.RIGHT_JOYSTICK);
        m_leftJoystickButton = new JoystickButton(m_leftJoystick, 1);
        m_rightJoystickButton = new JoystickButton(m_rightJoystick, 1);
        m_drivetrain = new Drivetrain();
        m_intake = new Intake(Constants.Intake.MOTOR_ID, Constants.Intake.SOLENOID_ID);
        m_tower = new Tower(Constants.Tower.MOTOR_ID);

        m_drivetrain.setDefaultCommand(new Drive(m_leftJoystick, m_rightJoystick, m_drivetrain));

        // TODO: This setup will only run robot-oriented while a button is held. This may not be
        // ideal is the drive ALWAYS wants to run in robot-oriented (maybe the IMU drift gets so
        // far off that field-oriented isn't useful)
        // m_rightJoystickButton.whenPressed(
        //     new InstantCommand(() -> m_drivetrain.setIsFieldOriented(false), m_drivetrain)
        // );
        // m_rightJoystickButton.whenReleased(
        //     new InstantCommand(() -> m_drivetrain.setIsFieldOriented(true), m_drivetrain)
        // );
        m_rightJoystickButton.whenPressed(new InstantCommand(() -> m_tower.loadCargo(), m_tower));
        m_rightJoystickButton.whenReleased(new InstantCommand(() -> m_tower.turnOffTower(), m_tower));
        //m_rightJoystickButton.whenPressed(new HomeSwerveModules(m_drivetrain), false);
        m_leftJoystickButton.whenPressed(new InstantCommand(() -> m_intake.toggleIntake(), m_intake));

    }

}
