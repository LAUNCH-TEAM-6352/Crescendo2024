// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

public class TestManipulatorIntake extends Command
{
    private final Manipulator manipulator;

    public TestManipulatorIntake(Manipulator manipulator)
    {
        this.manipulator = manipulator;
        
        addRequirements(manipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        manipulator.moveToIntakePosition();
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
        return false;
    }
}
