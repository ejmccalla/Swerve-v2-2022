package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.SwerveDriveSignal;
import frc.robot.subsystems.Drivetrain;

/**
 * Implements a command to drive the robot using driver inputs.
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

    @Override
    public void execute() {
        m_speedX = MathUtil.applyDeadband(m_rateLimiterX.calculate(m_rightJoystick.getX()), 0.05);
        m_speedX *= Constants.Driver.MAX_DRIVE_VELOCITY_MPS;
        m_speedY = MathUtil.applyDeadband(m_rateLimiterY.calculate(-m_rightJoystick.getY()), 0.05);
        m_speedY *= Constants.Driver.MAX_DRIVE_VELOCITY_MPS;
        m_speedZ = MathUtil.applyDeadband(m_rateLimiterZ.calculate(m_leftJoystick.getZ()), 0.05);
        m_speedZ *= Constants.Driver.MAX_ROTATION_VELOCITY_RPS;

        // TODO: Review the need for the PID which holds the rotational angle when no rotation is
        // called for

        //m_drivetrain.requestDrive(new SwerveDriveSignal(m_speedX, m_speedY, m_speedZ));
        m_drivetrain.requestDrive(new SwerveDriveSignal(0.0, 0.0, m_speedZ));

    }

}
