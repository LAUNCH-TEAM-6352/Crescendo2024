package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
/**
 * RunIntake intakes a note from the floor
 * move manipulator into intake position
 * start indexer running for intake 
 * tell Intake subsystem to intake
 */ 
public class RunIntake extends Command{
    private final Intake intake;

    private RunIntake(Intake intake)
    {
        this.intake = intake;
        //Specify required subsystems
        addRequirements(intake);
    }

    @Override
    public void initialize()
    {
        //  TODO: Move manipulator to intake position 
        //  TODO: Indexer running for intake 
        intake.intake();

    }
    @Override
    public void execute()
    {
        //Intentionally blank, PID Controllers are controlling motors
    }
    //Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted)
    { 
        intake.stop();
        // TODO: Indexer stop running for intake
        // TODO: Move manipulator?

    }
    //Returns true when the command should end
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
