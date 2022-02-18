// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drive = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  NetworkTable table;
  double[] areas;
  double[] defaultValue = new double[0];

  @Override
  public void robotInit() {
    table = NetworkTableInstance.getDefault().getTable("Table");
  }
  
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
    m_drive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    // // Get the x speed. We are inverting this because Xbox controllers return
    // // negative values when we push forward.
    // final var xSpeed = -m_speedLimiter.calculate(m_controller.getLeftY()) * Drivetrain.kMaxSpeed;

    // // Get the rate of angular rotation. We are inverting this because we want a
    // // positive value when we pull to the left (remember, CCW is positive in
    // // mathematics). Xbox controllers return positive values when you pull to
    // // the right by default.
    // final var rot = -m_rotLimiter.calculate(m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;

    // m_drive.drive(xSpeed, rot);

    // gets table values as doubles, returns 0 if table is empty 
    double linX = -table.getEntry("linearX").getDouble(0);
    double angZ = -table.getEntry("angularZ").getDouble(0);

    m_drive.drive(linX, angZ);


    // double[] linX = table.getEntry("linearX").getDoubleArray(defaultValue);

    // System.out.print("X: " );

    // for (double val : linX) {
    //   System.out.print(val + " ");
    // }

    // System.out.println();
  }
}
