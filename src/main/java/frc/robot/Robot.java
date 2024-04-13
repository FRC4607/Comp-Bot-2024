// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

public class Robot extends TimedRobot {
    private static BaseStatusSignal[] m_signalsToRefreshCaniv = {};
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    /**
     * Adds signals to be refreshed before every loop. The signals provided should
     * be on the CANivore's CAN bus.
     * 
     * @param signals The signals to be refreshed.
     */
    public static void addSignalsCaniv(BaseStatusSignal... signals) {
        BaseStatusSignal[] newSignals = new BaseStatusSignal[m_signalsToRefreshCaniv.length + signals.length];
        System.arraycopy(m_signalsToRefreshCaniv, 0, newSignals, 0, m_signalsToRefreshCaniv.length);
        System.arraycopy(signals, 0, newSignals, m_signalsToRefreshCaniv.length, signals.length);
        m_signalsToRefreshCaniv = newSignals;
    }

    @Override
    public void robotInit() {
        // SignalLogger.setPath("/media/sda1/");
        m_robotContainer = new RobotContainer();
        RobotController.setBrownoutVoltage(6.3);
        SignalLogger.start();
        DataLogManager.start("", "", 0.2);
    }

    @Override
    public void robotPeriodic() {
        // for (int i = 0; i < m_currents.length; i++) {
        //     m_currents[i] = m_pd.getCurrent(i);
        // }
        // m_currentsLog.append(m_currents);
        // m_energyLog.append(m_pd.getTotalEnergy());
        // m_voltageLog.append(m_pd.getVoltage());
        // m_tempLog.append(m_pd.getTemperature());
        // Refresh every signal before running loop
        BaseStatusSignal.refreshAll(m_signalsToRefreshCaniv);
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        LEDSubsystem.setDisabled();
    }

    @Override
    public void disabledPeriodic() {
        if (Math.abs(m_robotContainer.getArmPosition()) > 5){
            LEDSubsystem.setError();
        }else{
            LEDSubsystem.setDisabled();
        }
        
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        LEDSubsystem.setNeutral();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        LEDSubsystem.setNeutral();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
