// Copyright (c) FIRST and other WPILib contributors.
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.RobotContainer.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.LimelightHelpers;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {

        m_robotContainer.limelightCalibration();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.setLimelightUseMegaTag2(false);
        m_robotContainer.disabledPeriodicVisionUpdate();
        SmartDashboard.putBoolean("right", RobotContainer.right);
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.setLimelightUseMegaTag2(true);
        m_robotContainer.setLimelightIMUMode("limelight-four", 4); // simplify this logic 
        m_robotContainer.setLimelightIMUMode("limelight-five", 0);
        
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        
    }

    @Override
    public void teleopInit() {
        m_robotContainer.setLimelightUseMegaTag2(true);
        m_robotContainer.setLimelightIMUMode("limelight-four", 4);
        m_robotContainer.setLimelightIMUMode("limelight-five", 0);

        // if (RobotContainer.limelightBACK.estimate != null)
        //     RobotContainer.drivetrain.resetTranslation(RobotContainer.limelightBACK.estimate.pose.getTranslation());

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        LimelightHelpers.SetRobotOrientation(RobotContainer.limelightBACK.getLimelightName(), drivetrain.getCachedState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(RobotContainer.limelightFRONT.getLimelightName(), drivetrain.getCachedState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
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
