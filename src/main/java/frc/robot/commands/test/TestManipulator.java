// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.Manipulator;

/**
 * Tests the Manipulator subsystem.
 */
public class TestManipulator extends SequentialCommandGroup
{
    public TestManipulator(Manipulator manipulator)
    {
        addCommands(
            new TestManipulatorIntake(manipulator).withTimeout(TestConstants.manipulatorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestManipulatorAmp(manipulator).withTimeout(TestConstants.manipulatorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestManipulatorIntake(manipulator).withTimeout(TestConstants.manipulatorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestManipulatorClimb(manipulator).withTimeout(TestConstants.manipulatorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestManipulatorClimbOff(manipulator).withTimeout(TestConstants.manipulatorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestManipulatorIntake(manipulator).withTimeout(TestConstants.manipulatorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs)
        );
    }
}
