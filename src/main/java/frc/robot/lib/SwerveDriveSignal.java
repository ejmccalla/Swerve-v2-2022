package frc.robot.lib;

/**
 * Implement a swerve drive signal which captures the translational speeds and the rotational speed.
 */
public class SwerveDriveSignal {

    private final double m_speedX;
    private final double m_speedY;
    private final double m_speedRotational;

    public double getxSpeed() {
        return m_speedX;
    }

    public double getySpeed() {
        return m_speedY;
    }

    public double getRotationalSpeed() {
        return m_speedRotational;
    }

    /**
     * Constructor for a swerve drive signal.
     *
     * @param speedX the X-axis translational speed in meters-per-second
     * @param speedY the Y-axis translational speed in meters-per-second
     * @param speedRotation the rotational speed in radians-per-second
     */
    public SwerveDriveSignal(double speedX, double speedY, double speedRotation) {
        m_speedX = speedX;
        m_speedY = speedY;
        m_speedRotational = speedRotation;
    }
}
