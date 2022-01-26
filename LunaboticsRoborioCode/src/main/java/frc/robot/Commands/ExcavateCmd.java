package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Excavator;

public class ExcavateCmd extends CommandBase {

    Excavator excavator; 

    ExcavateCmd() {
        
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        excavator.excavate(1); //TODO: Placeholder Speed, want to use variable probs, 
                               // or have some fancy speed profile
    }

    @Override
    public void end(boolean interrupted) {
        excavator.stopExc();
    }
}
