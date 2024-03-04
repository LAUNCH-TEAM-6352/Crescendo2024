// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Shooter;

/**
 * ShootNoteIntoAmp shoots a note into the Amp
 * move manipulator into Amp position
 * start shooter
 * tell indexer subsystem to feed
 * determine what feedback will be provided to drivers during command execution
 */
public class ShootNoteIntoAmp extends Command
{
    private final Indexer indexer;
    private final Shooter shooter;
    private final Manipulator manipulator;
    
    private boolean hasShooterBeenFed;
    /**
     * 
     * 
     * @param indexer
     */
    public ShootNoteIntoAmp(Indexer indexer, Shooter shooter, Manipulator manipulator)
    {
        this.indexer = indexer;
        this.shooter = shooter;
        this.manipulator = manipulator;
        // Specify required subsystems
        addRequirements(indexer, shooter, manipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        manipulator.moveToAmpPosition();
        shooter.setAmpSpeed();
        hasShooterBeenFed = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if (shooter.isAtTargetVelocity() && !hasShooterBeenFed)
        {
            indexer.feed();
            hasShooterBeenFed = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        indexer.stop();
        manipulator.moveToIntakePosition();
        shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
