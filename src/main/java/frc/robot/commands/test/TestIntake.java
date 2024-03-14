// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.Intake;

/**
 * Tests the Intake subsystem.
 */
public class TestIntake extends SequentialCommandGroup
{
    public TestIntake(Intake intake)
    {
        addCommands(
            new TestIntakeIntake(intake).withTimeout(TestConstants.intakeMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestIntakeEject(intake).withTimeout(TestConstants.intakeMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs)
        );
    }
}
