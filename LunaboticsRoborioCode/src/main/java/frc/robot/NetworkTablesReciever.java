package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTablesReciever {

    public static void main(String[] args) {
        new NetworkTablesReciever().run();
    }
    
    public void run() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Table");
        //TODO: get entries 

        NetworkTableEntry x = table.getEntry("linearX");
        NetworkTableEntry z = table.getEntry("angularZ");
        // NetworkTableEntry drivePath = table.getEntry("drivePath");
        inst.startClient("130.215.211.73");
        // inst.startClientTeam(2022); // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
        while (true) {
          try {
            Thread.sleep(1000);
          } catch (InterruptedException ex) {
            System.out.println("interrupted");
            return;
          }
          //TODO: Add entry fields
          // double p = drivePath.getDouble(0.0);

          System.out.println("x: " + x + "\tz: " + z);
        }
      }
}
