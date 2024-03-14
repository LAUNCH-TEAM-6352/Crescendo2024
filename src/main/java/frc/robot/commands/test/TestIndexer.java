// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.Indexer;

/**
 * Tests the Indexer subsystem.
 */
public class TestIndexer extends SequentialCommandGroup
{
    public TestIndexer(Indexer indexer)
    {
        addCommands(
            new TestIndexerIntake(indexer).withTimeout(TestConstants.indexerMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestIndexerEject(indexer).withTimeout(TestConstants.indexerMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestIndexerFeed(indexer).withTimeout(TestConstants.indexerMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs)
        );
    }
}
