import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final CANSparkMax Winch1 = new CANSparkMax(,MotorType.kBrushless);
    private final CANSparkMax Winch2 = new CANSparkMax(,MotorType.kBrushless);
    private final CANSparkMax Winch3 = new CANSparkMax(,MotorType.kBrushless);
    private final CANSparkMax Slide1 = new CANSparkMax(,MotorType.kBrushless);
    private final CANSparkMax Slide2 = new CANSparkMax(,MotorType.kBrushless);
    private final Joystick m_stick = new Joystick(0);

    
    public void slideOut(){
        if(m_stick.getRawButton(6)){
            Slide1.set(1);
            Slide2.set(1);
          
        }else{
            Slide1.set(1);
            Slide2.set(1);
        }
    }
      public void slideIn(){
        if(m_stick.getRawButton(4)){
            Slide1.set(-1);
            Slide2.set(-1);
          
        }else{
            Slide1.set(1);
            Slide2.set(1);
        }
    }
        public void winch1InOut(){
            if(m_stick.getRawButton(7)){
                Winch1.set(1);
            }else if(m_stick.getRawButton(8)){
                Winch1.set(-1);
            }else{
                Winch1.set(1);
            }
    }
        public void winch2InOut(){
            if(m_stick.getRawButton(9)){
                Winch2.set(1);
            }else if(m_stick.getRawButton(10)){
                Winch2.set(-1);
            }else{
                Winch2.set(1);
            }
    }
        public void winch3InOut(){
            if(m_stick.getRawButton(11)){
                Winch1.set(1);
            }else if(m_stick.getRawButton(12)){
                Winch1.set(-1);
            }else{
                Winch1.set(1);
            }
    }
}
