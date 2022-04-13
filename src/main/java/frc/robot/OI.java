package frc.robot;
//imports
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;//allows extra joystic if needed


public class OI {
    //class declarations
    private XboxController m_stick;
    //constructo
    public OI(){
    m_stick = new XboxController(0);
    }
    //methods
    public double LeftX  (){  return m_stick.getRawAxis(0);  }
    public double LeftY  (){  return m_stick.getRawAxis(1);  }
    public double RightX (){  return m_stick.getRawAxis(4);  }
    public double RightY (){  return m_stick.getRawAxis(5);  }
    public boolean XBut(){ return m_stick.getXButtonPressed();}
    public boolean YBut(){ return m_stick.getYButtonPressed();}
    public boolean ABut(){ return m_stick.getAButtonPressed();}
    public boolean BBut(){ return m_stick.getBButtonPressed();}
}
