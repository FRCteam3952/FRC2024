// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    public Robot() {
        super();
    }

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // this.initializeAdvantageKit();

        this.robotContainer = new RobotContainer();
        this.robotContainer.onRobotInit();
    }

    private void initializeAdvantageKit() {
        // https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md#robot-configuration
        Logger.recordMetadata("3952", "Crescendo_Robot");

        if(isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs/"));
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        this.robotContainer.onRobotPeriodic();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        this.robotContainer.getAutonomousCommand().schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        this.robotContainer.onTeleopInit();
    }

    @Override
    public void teleopPeriodic() {
        this.robotContainer.onTeleopPeriodic();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
