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
        NetworkTable table = inst.getTable("datatable");
        //TODO: get entries 
        NetworkTableEntry drivePath = table.getEntry("drivePath");
        inst.startClientTeam(2022); //TODO: update number // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
        while (true) {
          try {
            Thread.sleep(1000);
          } catch (InterruptedException ex) {
            System.out.println("interrupted");
            return;
          }
          //TODO: Add entry fields
          double p = drivePath.getDouble(0.0);
          System.out.println("path: " + p);
        }
      }
}
