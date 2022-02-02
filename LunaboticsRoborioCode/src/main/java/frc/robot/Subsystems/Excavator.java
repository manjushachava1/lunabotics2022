package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Excavator extends SubsystemBase{

    private TalonSRX excTalon; 
    private TalonSRX rotateTalon;
    private TalonSRX traverseTalon; 

    private static final int DIG_POS = 200; //TODO: set correct pos
    private static final int TRAVEL_POS = 800; //TODO: set correct pos

    public Excavator(){
        excTalon = new TalonSRX(Constants.EXC_TALON_ID);
        rotateTalon = new TalonSRX(Constants.EXC_ROTATE_ID);
        traverseTalon = new TalonSRX(Constants.EXC_TRAVERSE_ID);
    }   

    // spins excavator belt
    public void excavate(double speed){
        excTalon.set(ControlMode.PercentOutput, speed);
    }

    // stops excavator belt
    public void stopExc(){
        excTalon.set(ControlMode.PercentOutput, 0);
    }

    // rotates excavator to digging position
    public void rotateFwd(){
        rotateTalon.set(ControlMode.Position, DIG_POS);
    }

    // rotates excavator to travel position
    public void rotateBkwd(){
        rotateTalon.set(ControlMode.Position, TRAVEL_POS);
    }

    public void stopRotate(){
        rotateTalon.set(ControlMode.PercentOutput, 0);
    }

    // traverses excavator up and down
    public void traverse(double speed){
        traverseTalon.set(ControlMode.PercentOutput, speed); 
        // TODO: could also do this with position setpoints?
    }
}
