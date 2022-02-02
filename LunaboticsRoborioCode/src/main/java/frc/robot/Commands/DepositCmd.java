package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Storage;

public class DepositCmd extends CommandBase {
    Storage storage; 

    DepositCmd() {
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        storage.deposit(1);; //TODO: Placeholder Speed, want to use variable probs, 
                            // or have some fancy speed profile
    }

    @Override
    public void end(boolean interrupted) {
        storage.stopStoBelt();
    }
    
}
