package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTablesListener {

    public static void main(String[] args) {
        new NetworkTablesListener().run();
     }
  
     public void run() {
        //get the default instance of NetworkTables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
  
        //get a reference to the subtable called "datatable"
        NetworkTable datatable = inst.getTable("datatable");
  
        //get a reference to key in "datatable" called "Y"
        NetworkTableEntry drivePath = datatable.getEntry("drivePath");
        inst.startClientTeam(2022);
  
        //add an entry listener for changed values of "Y", the lambda ("->" operator)
        //defines the code that should run when "Y" changes
        drivePath.addListener(event -> {
           System.out.println("path changed: " + event.getEntry().getValue());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  
        try {
           Thread.sleep(10000);
        } catch (InterruptedException ex) {
           System.out.println("Interrupted");
           Thread.currentThread().interrupt();
           return;
        }
     }
    
}

// Sample Listener (copy to above then uncomment)
      //   //add an entry listener for changed values of "X", the lambda ("->" operator)
      //   //defines the code that should run when "X" changes
      //   datatable.addEntryListener("X", (table, key, entry, value, flags) -> {
      //      System.out.println("X changed value: " + value.getValue());
      //   }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);