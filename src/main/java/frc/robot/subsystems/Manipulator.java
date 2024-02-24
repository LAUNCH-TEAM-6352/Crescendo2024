// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.PneumaticsConstants;

public class Manipulator extends SubsystemBase
{
    private final DoubleSolenoid noteSolenoid = new DoubleSolenoid(PneumaticsConstants.moduleType,
                    ManipulatorConstants.noteSolenoidForwardChannel, ManipulatorConstants.noteSolenoidReverseChannel);
    private final DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsConstants.moduleType,
                    ManipulatorConstants.climbingSolenoidForwardChannel,
                    ManipulatorConstants.climbingSolenoidReverseChannel);

    /** Creates a new manipulator. */
    public Manipulator()
    {
    }

    public void moveToAmpPosition()
    {
        noteSolenoid.set(Value.kReverse);
    }
    
    public void moveToClimbingPosition()
    {
        noteSolenoid.set(Value.kReverse);
    }

    public void moveToSpeakerPosition()
    {
        noteSolenoid.set(Value.kForward);
    }

    public void moveToIntakePosition()
    {
        noteSolenoid.set(Value.kForward);
    }

    public void climb()
    {
        climberSolenoid.set(Value.kForward);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
