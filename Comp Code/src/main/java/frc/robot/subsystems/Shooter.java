package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    public CANSparkMax frontMotor;
    public CANSparkMax backMotor;
    public RelativeEncoder frontEncoder;
    public RelativeEncoder backEncoder;

    private SparkMaxPIDController PIDController;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double maxRPM;

    private double SETPOINT = 0;
    
    public Shooter(){
        frontMotor = new CANSparkMax(Constants.frontShootCAN, MotorType.kBrushless);
        backMotor = new CANSparkMax(Constants.backShootCAN, MotorType.kBrushless);
        
        frontEncoder = frontMotor.getEncoder();
        backEncoder = backMotor.getEncoder();

        backMotor.setInverted(false);
        frontMotor.setInverted(false);
        //backMotor.follow(frontMotor);
        //PIDController = frontMotor.getPIDController();

        kP = 0.0003;
        kI = 0;
        kD = 0.0005;
        kIz = 0;
        kFF = 0.00019231;
        kMaxOutput = 1;
        kMinOutput = -1;

        maxRPM = 5200;
        /*
        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setIZone(kIz);
        PIDController.setFF(kFF);
        PIDController.setOutputRange(kMinOutput, kMaxOutput);
        */
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Motor Speed", -frontEncoder.getVelocity());
        SmartDashboard.putNumber("Back Motor Speed", backEncoder.getVelocity());

        /*
        double setPoint = SETPOINT*maxRPM;
        PIDController.setReference(setPoint, ControlType.kVelocity);
        */
        //0.65 for up against wall
        frontMotor.set(0.80);
        backMotor.set(-0.65);
        //SmartDashboard.putNumber("SetPoint", setPoint);

    }

    public void spinUP(double velocity){
        SETPOINT = velocity/maxRPM;
    }
}
