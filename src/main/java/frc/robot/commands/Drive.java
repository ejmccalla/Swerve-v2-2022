package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.SwerveDriveSignal;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.StateType;

/**
 * Implements a command to drive the robot using driver inputs.
 * 
 * <p>This command is intended to be the default subsystem command. As such, there is no condition that ends the command and
 * there's nothing to do when the command is finished (even if by being interrupted).
 */
public class Drive extends CommandBase {

    private Drivetrain m_drivetrain;
    private Joystick m_leftJoystick;
    private Joystick m_rightJoystick;
    private SlewRateLimiter m_rateLimiterX;
    private SlewRateLimiter m_rateLimiterY;
    private SlewRateLimiter m_rateLimiterZ;
    private double m_speedX;
    private double m_speedY;
    private double m_speedZ;

    /**
     * Constructor for the drivetrain drive command.
     *
     * @param leftJoystick the left joystick controls the rotational velocity
     * @param rightJoystick the right joystick conrols the X and Y translational velocities
     * @param drivetrain uses the drivetrain for drive encoder feedback to the smart dashboard
     */
    public Drive(Joystick leftJoystick, Joystick rightJoystick, Drivetrain drivetrain) {
        m_leftJoystick = leftJoystick;
        m_rightJoystick = rightJoystick;
        m_drivetrain = drivetrain;

        m_rateLimiterX = new SlewRateLimiter(5);
        m_rateLimiterY = new SlewRateLimiter(5);
        m_rateLimiterZ = new SlewRateLimiter(5);

        addRequirements(m_drivetrain);
    }

    /**
     * Update the drivetrain state to indicate the subysystem is being driven manually.
     * 
     * <p>This command should be scheduled as non-interruptable.
     */
    @Override
    public void initialize() {
        m_drivetrain.updateState(StateType.Driving);
    }

    /**
     * Compose the swerve drive signal.
     * 
     * <p>Each of the driver inputs (X and Y translational velocity and rotational velocity) are rate limited and have an
     * appied deadband. These values should be set based on driver feedback.
     */
    @Override
    public void execute() {
        m_speedX = MathUtil.applyDeadband(m_rateLimiterX.calculate(m_rightJoystick.getX()), 0.05);
        m_speedX *= Constants.Driver.MAX_DRIVE_VELOCITY_MPS;
        m_speedY = MathUtil.applyDeadband(m_rateLimiterY.calculate(-m_rightJoystick.getY()), 0.05);
        m_speedY *= Constants.Driver.MAX_DRIVE_VELOCITY_MPS;
        m_speedZ = MathUtil.applyDeadband(m_rateLimiterZ.calculate(m_leftJoystick.getZ()), 0.05);
        m_speedZ *= Constants.Driver.MAX_ROTATION_VELOCITY_RPS;

        // TODO: Review the need for the PID which holds the rotational angle when no rotation is commanded.
        //m_drivetrain.setDesiredModulesState(new SwerveDriveSignal(m_speedX, m_speedY, m_speedZ));
        m_drivetrain.setIdleModules();


    }

}
