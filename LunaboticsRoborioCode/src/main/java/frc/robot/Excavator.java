package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Excavator extends SubsystemBase{

    public static final TalonSRX excTalon = new TalonSRX(Constants.EXC_TALON_ID);

    public Excavator(){

    }

    public void excavate(double speed){
        excTalon.set(ControlMode.PercentOutput, speed);
    }

    public void stopExc(){
        excTalon.set(ControlMode.PercentOutput, 0);
    }
}
