package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Excavator;

public class StopExcavateCmd extends CommandBase{
    
    Excavator excavator; 

    StopExcavateCmd() {}

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        excavator.stopExc();
    }

    @Override
    public void end(boolean interrupted) {
        excavator.stopExc();
    }
}
