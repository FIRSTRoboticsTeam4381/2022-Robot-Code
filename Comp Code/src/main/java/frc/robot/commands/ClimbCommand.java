package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends CommandBase{
    
    private Climb climb;

    public ClimbCommand(Climb climb){
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute(){
    }
}
