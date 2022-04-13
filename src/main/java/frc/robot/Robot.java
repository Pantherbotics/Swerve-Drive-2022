/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*This program modified by CWS 8/4/2021,   This will track the motion of an analog
 sensor which rotates beyond 360 deg.
 Note the pot is connected directly to the Robo Rio analog in
    The Talon sensor collection can measure the value without roll up, but it is too slow at 150msec.
    The talon PID cannot accomodate the 0/360 discontinuity
    Spark controls the drive motor (Neo) in speed mode.  
  
 */
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
 // Class Declarations
  Drivex m_drive;
  OI ThisOI;
  
  



  @Override
  public void robotInit() {
    //instantiate classes
    m_drive = new Drivex();
    
  
  }
  // void telepPeriodicInit(){}//just here to override the default Init
  //Talon param setup must be done in  Robot Init.   

  @Override
  public void teleopPeriodic() 
  {
    
    m_drive.run();//the parameter is the drive mode used int Drive
   
  }
}
/*Note axis for xbox controller
0=left X
1=left Y 
2=left Trigger
3=Right Trigger
4=Right X
5=Right Y
*/
