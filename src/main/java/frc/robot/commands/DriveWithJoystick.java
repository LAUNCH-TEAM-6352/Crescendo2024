package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoystick extends Command
{
    private final DriveTrain DriveTrain;
    private final Joystick driverController;
    private final SendableChooser<Boolean> driveOrientationChooser;

    public DriveWithJoystick(DriveTrain driveTrain, Joystick driverController,
                    SendableChooser<Boolean> driveOrientationChooser)
    {
        this.DriveTrain = driveTrain;
        this.driverController = driverController;
        this.driveOrientationChooser = driveOrientationChooser;

        // Specify subsystem dependencies (if any)
        addRequirements(driveTrain);
    }

    @Override
    public void initialize()
    {
        // Perform any initialization if needed
    }

    @Override
    public void execute()
    {
        // Get joystick inputs
        double joystickX = driverController.getX();
        double joystickY = -driverController.getY(); // Invert Y axis if needed
        double joystickRotation = driverController.getTwist();

        // Apply deadband and sensitivity adjustments
        joystickX = applyDeadbandAndSensitivity(joystickX);
        joystickY = applyDeadbandAndSensitivity(joystickY);
        joystickRotation = applyDeadbandAndSensitivity(joystickRotation);

        // Drives according to linear speed, rotational speed, and if field is relative (true for now)
        double speedX = joystickY * SwerveConstants.maximumLinearVelocityMps;
        double speedY = -joystickX * SwerveConstants.maximumLinearVelocityMps;
        double rotationRate = -joystickRotation * SwerveConstants.maximumRotationRateRps;
        Translation2d translationSpeed = new Translation2d(speedX, speedY);
        DriveTrain.drive(translationSpeed, rotationRate, driveOrientationChooser.getSelected());
    }

    private double applyDeadbandAndSensitivity(double input)
    {
        return (Math.abs(input) < OperatorConstants.joystickDeadband) ? 0.0 : Math.pow(input, 3);
    }

    @Override
    public void end(boolean interrupted)
    {
        // Perform any actions when the command ends
    }

    @Override
    public boolean isFinished()
    {
        // This command is intended to run continuously during teleop
        return false;
    }
}
