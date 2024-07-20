// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DashboardConstants.IntakeKeys;
import frc.robot.Constants.IntakeConstants.PIDConstants;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Intake extends SubsystemBase
{
    private final CANSparkMax largeRollerMotor = new CANSparkMax(IntakeConstants.largeRollerMotorChannel,
        MotorType.kBrushless);

    private final CANSparkMax smallRollerMotor = new CANSparkMax(IntakeConstants.smallRollerMotorChannel,
        MotorType.kBrushless);

        private final DigitalInput noteSensor = new DigitalInput(IntakeConstants.opticalSensorPort);
        private final AnalogInput noteSensorAnalog = new AnalogInput(IntakeConstants.opticalSensorPortAnalog);
    
    private final XboxController gamepad;

    /** Creates a new Intake. */
    public Intake(XboxController gamepad)
    {
        // Apply configuration common to both large and small roller motors:
        for (CANSparkMax motor : new CANSparkMax[] { largeRollerMotor, smallRollerMotor })
        {
            motor.restoreFactoryDefaults();
            motor.clearFaults();
            motor.setIdleMode(IntakeConstants.motorIdleMode);

            var pidController = motor.getPIDController();
            pidController.setP(PIDConstants.kP);
            pidController.setI(PIDConstants.kI);
            pidController.setD(PIDConstants.kD);
            pidController.setIZone(PIDConstants.kIZ);
            pidController.setFF(PIDConstants.kFF);
            pidController.setOutputRange(PIDConstants.minOutput, PIDConstants.maxOutput);
        }

        // Apply configuration unique to large and small roller motors:
        largeRollerMotor.setInverted(IntakeConstants.isLargeRollerMotorInverted);
        smallRollerMotor.setInverted(IntakeConstants.isSmallRollerMotorInverted);
        this.gamepad = gamepad;
    }

    public void intake()
    {
        largeRollerMotor.getPIDController()
            .setReference(
                SmartDashboard.getNumber(IntakeKeys.largeRollerIntakeRpm, IntakeConstants.largeRollerMotorIntakeRpm),
                CANSparkBase.ControlType.kVelocity);

        smallRollerMotor.getPIDController()
            .setReference(
                SmartDashboard.getNumber(IntakeKeys.smallRollerIntakeRpm, IntakeConstants.smallRollerMotorIntakeRpm),
                CANSparkBase.ControlType.kVelocity);
    }

    public void eject()
    {
        largeRollerMotor.getPIDController()
            .setReference(
                SmartDashboard.getNumber(IntakeKeys.largeRollerEjectRpm, IntakeConstants.largeRollerMotorEjectRpm),
                CANSparkBase.ControlType.kVelocity);

        smallRollerMotor.getPIDController()
            .setReference(
                SmartDashboard.getNumber(IntakeKeys.smallRollerEjectRpm, IntakeConstants.smallRollerMotorEjectRpm),
                CANSparkBase.ControlType.kVelocity);
    }

    public void stop()
    {
        largeRollerMotor.stopMotor();
        smallRollerMotor.stopMotor();
    }

    public boolean hasNote()
    {
        var voltage = noteSensorAnalog.getVoltage();
        SmartDashboard.putNumber(IntakeKeys.noteSensorVoltage, voltage);
        return voltage > IntakeConstants.noteSensorVoltageThreshold;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        var hasNote = hasNote();
        gamepad.setRumble(RumbleType.kLeftRumble, hasNote ? 1 : 0);
        SmartDashboard.putBoolean(IntakeKeys.hasNote, hasNote);
        SmartDashboard.putNumber("Intake/Large/RPM", largeRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake/Small/RPM", smallRollerMotor.getEncoder().getVelocity());

    }
}
