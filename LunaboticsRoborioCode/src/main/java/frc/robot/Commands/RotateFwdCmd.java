package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Excavator;

public class RotateFwdCmd extends CommandBase{

    Excavator excavator;     

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        excavator.rotateFwd();
    }

    @Override
    public void end(boolean interrupted) {
        excavator.stopRotate();
    }
    
}
