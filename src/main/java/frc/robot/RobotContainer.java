package frc.robot;

import frc.robot.subsystems.Drivetrain;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;

/**
 * The RobotContainer class contains all of the subsystems, commands, and defines how the users
 * interface with the robot.
 */
public class RobotContainer {

    // private final Joystick m_leftJoystick =
    //     new Joystick(Constants.DriverStation.LEFT_JOYSTICK);
    // private final Joystick m_rightJoystick =
    //     new Joystick(Constants.DriverStation.RIGHT_JOYSTICK);
    // private final SendableChooser<Command> m_autonomousSelector = new SendableChooser<>();
    
    public final Drivetrain m_drivetrain;


    public RobotContainer() {
        m_drivetrain = new Drivetrain();
    }

}
