// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Shooter;

/**
 * ShootNoteIntoAmp shoots a note into the Amp
 * move manipulator into speaker position
 * start shooter
 * tell indexer subsystem to feed
 * determine what feedback will be provided to drivers during command execution
 */
public class ShootNoteIntoSpeaker extends Command
{
    private final Indexer indexer;
    private final Shooter shooter;
    private final Manipulator manipulator;
    private String timeoutKey = null;
    private long stopTime;

    private boolean hasShooterBeenFed;

    /**
     *
     * 
     * @param indexer
     */
    public ShootNoteIntoSpeaker(Indexer indexer, Shooter shooter, Manipulator manipulator)
    {
        this.indexer = indexer;
        this.shooter = shooter;
        this.manipulator = manipulator;
        // Specify required subsystems
        addRequirements(indexer, shooter, manipulator);
    }

    public ShootNoteIntoSpeaker(Indexer indexer, Shooter shooter, Manipulator manipulator, String timeoutKey)
    {
        this(indexer, shooter, manipulator);
        this.timeoutKey = timeoutKey;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        manipulator.moveToSpeakerPosition();
        shooter.setSpeakerSpeed();
        hasShooterBeenFed = false;

        stopTime = timeoutKey == null
            ? Long.MAX_VALUE
            : RobotController.getFPGATime() + (long) (Constants.microsecondsPerSecond * SmartDashboard.getNumber(timeoutKey, 180));
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
        return RobotController.getFPGATime() >= stopTime;
    }
}
