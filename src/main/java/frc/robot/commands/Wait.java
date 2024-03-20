// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Wait extends Command
{
    private String dashboardKey = null;
    private long stopTime;

    /** Creates a new Wait. */
    public Wait(String dashboardKey)
    {
        this.dashboardKey = dashboardKey;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        stopTime = RobotController.getFPGATime() + (long) (SmartDashboard.getNumber(dashboardKey, 0) * 1000000);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return RobotController.getFPGATime() >= stopTime;
    }
}
