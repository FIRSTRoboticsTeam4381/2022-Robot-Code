package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndex;

public class IntakeIndexCommand extends CommandBase{
    
    private IntakeIndex intakeIndex;

    public IntakeIndexCommand(IntakeIndex intakeIndex){
        this.intakeIndex = intakeIndex;
        addRequirements(intakeIndex);
    }

    @Override
    public void execute(){
        //intakeIndex.zeroIndex();
        //intakeIndex.zeroIntake();
    }
}
