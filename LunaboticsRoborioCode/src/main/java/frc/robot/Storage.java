package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Storage {
    
    public static final TalonSRX stoTalon = new TalonSRX(Constants.EXC_TALON_ID);

    public Storage(){

    }

    public void deposit(double speed){
        stoTalon.set(ControlMode.PercentOutput, speed);
    }

    public void stopStoBelt(){
        stoTalon.set(ControlMode.PercentOutput, 0);
    }
}

