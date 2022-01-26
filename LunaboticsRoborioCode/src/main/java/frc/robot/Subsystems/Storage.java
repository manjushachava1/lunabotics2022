package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Storage {
    
    public TalonSRX stoTalon;

    public Storage(){
        stoTalon = new TalonSRX(Constants.EXC_TALON_ID);
    }

    public void deposit(double speed){
        stoTalon.set(ControlMode.PercentOutput, speed);
    }

    public void stopStoBelt(){
        stoTalon.set(ControlMode.PercentOutput, 0);
    }
}

