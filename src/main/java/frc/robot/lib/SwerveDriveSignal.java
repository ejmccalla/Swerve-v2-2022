package frc.robot.lib;

/**
 * Implement a swerve drive signal which includes the translational speeds, the rotational speed,
 * and a flag for field/robot oriented control.
 */
public class SwerveDriveSignal {

    private final double m_speedX;
    private final double m_speedY;
    private final double m_speedRotational;
    private final boolean m_isFieldOriented;

    public double getxSpeed() {
        return m_speedX;
    }

    public double getySpeed() {
        return m_speedY;
    }

    public double getRotationalSpeed() {
        return m_speedRotational;
    }

    public boolean getIsFieldOriented() {
        return m_isFieldOriented;
    }

    /**
     * Constructor for a swerve drive signal.
     *
     * @param speedX the X-axis translational speed in meters-per-second
     * @param speedY the Y-axis translational speed in meters-per-second
     * @param speedRotation the rotational speed in radians-per-second
     * @param isFieldOriented a flag indicating field oriented
     */
    public SwerveDriveSignal(double speedX, double speedY, double speedRotation,
                             boolean isFieldOriented) {
        m_speedX = speedX;
        m_speedY = speedY;
        m_speedRotational = speedRotation;
        m_isFieldOriented = isFieldOriented;
    }
}
