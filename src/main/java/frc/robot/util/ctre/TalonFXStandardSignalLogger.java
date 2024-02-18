package frc.robot.util.ctre;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Robot;

/**
 * A class that manages the logging and storage of useful signals of a single
 * TalonFX object.
 */
public class TalonFXStandardSignalLogger {
    private final TalonFX m_device;

    public final StatusSignal<Double> m_pos;
    private final DoubleLogEntry m_posLog;
    public final StatusSignal<Double> m_torque;
    private final DoubleLogEntry m_torqueLog;
    public final StatusSignal<Double> m_velocity;
    private final DoubleLogEntry m_velocityLog;
    public final StatusSignal<Double> m_acceleration;
    private final DoubleLogEntry m_accelerationLog;

    public final StatusSignal<Double> m_deviceTempSecondary;
    private final DoubleLogEntry m_deviceTempSecondaryLog;
    public final StatusSignal<Double> m_deviceTemp;
    private final DoubleLogEntry m_deviceTempLog;
    public final StatusSignal<Double> m_processorTemp;
    private final DoubleLogEntry m_processorTempLog;

    public final StatusSignal<Boolean> m_bootDuringEnable;
    private final BooleanLogEntry m_bootDuringEnableLog;
    public final StatusSignal<Boolean> m_hardware;
    private final BooleanLogEntry m_hardwareLog;
    public final StatusSignal<Boolean> m_deviceTempFault;
    private final BooleanLogEntry m_deviceTempFaultLog;
    public final StatusSignal<Boolean> m_procTempFault;
    private final BooleanLogEntry m_procTempFaultLog;

    /**
     * Creates a new TalonFXStandardSignalLogger.
     * 
     * @param device The {@link com.ctre.phoenix6.hardware.TalonFX} to use.
     * @param prefix A string that will be prefixed to the various log entries.
     *               Should not end with a trailing slash.
     */
    public TalonFXStandardSignalLogger(TalonFX device, String prefix) {
        m_device = device;

        DataLog managedLog = DataLogManager.getLog();

        m_pos = device.getPosition();
        m_pos.setUpdateFrequency(50.0);
        m_posLog = new DoubleLogEntry(managedLog, prefix + "/pos");
        m_torque = device.getTorqueCurrent();
        m_torque.setUpdateFrequency(50.0);
        m_torqueLog = new DoubleLogEntry(managedLog, prefix + "/torque");
        m_velocity = device.getVelocity();
        m_velocity.setUpdateFrequency(50.0);
        m_velocityLog = new DoubleLogEntry(managedLog, prefix + "/vel");
        m_acceleration = device.getVelocity();
        m_acceleration.setUpdateFrequency(50.0);
        m_accelerationLog = new DoubleLogEntry(managedLog, prefix + "/accel");

        m_deviceTempSecondary = device.getAncillaryDeviceTemp();
        m_deviceTempSecondary.setUpdateFrequency(4.0);
        m_deviceTempSecondaryLog = new DoubleLogEntry(managedLog, prefix + "/temp_secondary");
        m_deviceTemp = device.getDeviceTemp();
        m_deviceTemp.setUpdateFrequency(4.0);
        m_deviceTempLog = new DoubleLogEntry(managedLog, prefix + "/temp");
        m_processorTemp = device.getProcessorTemp();
        m_processorTemp.setUpdateFrequency(4.0);
        m_processorTempLog = new DoubleLogEntry(managedLog, prefix + "/temp_proc");

        m_bootDuringEnable = device.getStickyFault_BootDuringEnable();
        m_bootDuringEnable.setUpdateFrequency(4.0);
        m_bootDuringEnableLog = new BooleanLogEntry(managedLog, prefix + "/faults/boot_during_enable");
        m_hardware = device.getStickyFault_Hardware();
        m_hardware.setUpdateFrequency(4.0);
        m_hardwareLog = new BooleanLogEntry(managedLog, prefix + "/faults/hardware");
        m_deviceTempFault = device.getStickyFault_DeviceTemp();
        m_deviceTempFault.setUpdateFrequency(4.0);
        m_deviceTempFaultLog = new BooleanLogEntry(managedLog, prefix + "/faults/device_temp");
        m_procTempFault = device.getStickyFault_ProcTemp();
        m_procTempFault.setUpdateFrequency(4.0);
        m_procTempFaultLog = new BooleanLogEntry(managedLog, prefix + "/faults/proc_temp");

        Robot.addSignals(
                m_pos,
                m_torque,
                m_velocity,
                m_acceleration,
                m_deviceTempSecondary,
                m_deviceTemp,
                m_processorTemp,
                m_bootDuringEnable,
                m_hardware,
                m_deviceTempFault,
                m_procTempFault);
    }

    /**
     * Writes the current values of the signals to the log with their timestamps.
     * This method should be called in the {@code periodic} method of a
     * {@link edu.wpi.first.wpilibj2.command.Subsystem}.
     */
    public void log() {
        m_posLog.append(m_pos.getValueAsDouble(), (long) (m_pos.getTimestamp().getTime() * 1e6));
        m_torqueLog.append(m_torque.getValueAsDouble(), (long) (m_torque.getTimestamp().getTime() * 1e6));
        m_velocityLog.append(m_velocity.getValueAsDouble(), (long) (m_velocity.getTimestamp().getTime() * 1e6));
        m_accelerationLog.append(m_acceleration.getValueAsDouble(),
                (long) (m_acceleration.getTimestamp().getTime() * 1e6));
        m_deviceTempLog.append(m_deviceTemp.getValueAsDouble(), (long) (m_deviceTemp.getTimestamp().getTime() * 1e6));
        m_deviceTempSecondaryLog.append(m_deviceTempSecondary.getValueAsDouble(),
                (long) (m_deviceTempSecondary.getTimestamp().getTime() * 1e6));
        m_processorTempLog.append(m_processorTemp.getValueAsDouble(),
                (long) (m_processorTemp.getTimestamp().getTime() * 1e6));

        m_bootDuringEnableLog.append(m_bootDuringEnable.getValue(),
                (long) (m_bootDuringEnable.getTimestamp().getTime() * 1e6));
        if (m_bootDuringEnable.getValue()) {
            m_device.clearStickyFault_BootDuringEnable();
        }
        m_hardwareLog.append(m_hardware.getValue(), (long) (m_hardware.getTimestamp().getTime() * 1e6));
        if (m_bootDuringEnable.getValue()) {
            m_device.clearStickyFault_Hardware();
        }
        m_deviceTempFaultLog.append(m_deviceTempFault.getValue(),
                (long) (m_deviceTempFault.getTimestamp().getTime() * 1e6));
        if (m_bootDuringEnable.getValue()) {
            m_device.clearStickyFault_DeviceTemp();
        }
        m_procTempFaultLog.append(m_procTempFault.getValue(), (long) (m_procTempFault.getTimestamp().getTime() * 1e6));
        if (m_bootDuringEnable.getValue()) {
            m_device.clearStickyFault_ProcTemp();
        }
    }
}
