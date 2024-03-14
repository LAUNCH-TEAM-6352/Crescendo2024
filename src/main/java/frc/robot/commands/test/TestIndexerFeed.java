// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class TestIndexerFeed extends Command
{
    private final Indexer indexer;

    public TestIndexerFeed(Indexer indexer)
    {
        this.indexer = indexer;
        
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        indexer.feed();
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
        indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
