package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;

/**
 * The RobotContainer class contains all of the subsystems, commands, and defines how the users interface with the robot.
 */
public class RobotContainer {

    private final Joystick m_leftJoystick;
    private final Joystick m_rightJoystick;
    private final Joystick m_driverButtonBoard;
    // private final JoystickButton m_leftJoystickButton;
    private final JoystickButton m_rightJoystickButton;
    private final JoystickButton m_driverButtonBoardLeft;
    // private final JoystickButton m_driverButtonBoardRight;
    private final ParallelCommandGroup m_startIntakeAndShoot;
    private final ParallelCommandGroup m_stopIntakeAndShoot;
    public final Drivetrain m_drivetrain;
    public final Intake m_intake;
    public final Tower m_tower;
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                   PUBLIC METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

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
        m_driverButtonBoard = new Joystick(Constants.DriverStation.DRIVER_BUTTON_BOARD);
        // m_leftJoystickButton = new JoystickButton(m_leftJoystick, 1);
        m_rightJoystickButton = new JoystickButton(m_rightJoystick, 1);
        m_driverButtonBoardLeft = new JoystickButton(m_driverButtonBoard, 2);
        // m_driverButtonBoardRight = new JoystickButton(m_driverButtonBoard, 3);

        m_drivetrain = new Drivetrain();
        m_intake = new Intake();
        m_tower = new Tower();

        m_drivetrain.setDefaultCommand(new Drive(m_leftJoystick, m_rightJoystick, m_drivetrain));

        m_startIntakeAndShoot = new ParallelCommandGroup(new InstantCommand(() -> m_tower.loadCargo(), m_tower),
                                                         new InstantCommand(() -> m_intake.extendIntake(), m_intake));
        m_stopIntakeAndShoot = new ParallelCommandGroup(new InstantCommand(() -> m_tower.turnOffTower(), m_tower),
                                                        new InstantCommand(() -> m_intake.retractIntake(), m_intake));

        m_rightJoystickButton.whenPressed(new InstantCommand(() -> m_drivetrain.toggleFieldOriented(), m_drivetrain));
        m_driverButtonBoardLeft.whenPressed(m_startIntakeAndShoot);
        m_driverButtonBoardLeft.whenReleased(m_stopIntakeAndShoot);

        m_autoChooser.setDefaultOption("Auto 1", new InstantCommand(() -> m_tower.loadCargo(), m_tower));
        m_autoChooser.addOption("Auto 2", new InstantCommand(() -> m_tower.unloadCargo(), m_tower));


    }

}
