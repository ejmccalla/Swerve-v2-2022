package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Update manager class borrowed from FRC 2910. */
public final class UpdateManager {

    /** Each child must implement the Update method. */
    @FunctionalInterface
    public interface Updatable {
        void update(double time, double dt);
    }

    public final List<Updatable> m_updatables = new ArrayList<>();
    private double m_lastTimestamp = 0.0;
    private final Notifier m_updaterThread = new Notifier(() -> {
        synchronized (UpdateManager.this) {
            final double timestamp = Timer.getFPGATimestamp();
            final double dt = timestamp - m_lastTimestamp;
            m_lastTimestamp = timestamp;
            m_updatables.forEach(s -> s.update(timestamp, dt));
        }
    });

    public void startLoop(double period) {
        m_updaterThread.startPeriodic(period);
    }

    public void stopLoop() {
        m_updaterThread.stop();
    }

    public UpdateManager(Updatable... updatables) {
        this(Arrays.asList(updatables));
    }

    public UpdateManager(List<Updatable> updatables) {
        this.m_updatables.addAll(m_updatables);
    }

}