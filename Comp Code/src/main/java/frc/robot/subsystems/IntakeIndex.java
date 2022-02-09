public class IntakeIndex {
    private final CANSparkMax Intake = new CANSparkMax(52,MotorType.kBrushless);
    private final CANSparkMax Index = new CANSparkMax(51,MotorType.kBrushless);
    private final Joystick m_stick = new Joystick(0);

}
public void intake {
    if(m_stick.getRawButton(3)){
        Intake.set(1);
    }else{
        Intake.set(0)
    }
}
public void index {
    if(m_stick.getRawButton(4)){
        Index.set(0.5);
    }else{
        Index.set(0);
    }
}