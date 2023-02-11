package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.armJoint;
import frc.robot.Constants.Swerve.armShoulder;
public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax shoulderMotor; 
    private CANSparkMax jointMotor;
    public ArmSubsystem() {
        shoulderMotor = new CANSparkMax(armShoulder.rotMotorID, MotorType.kBrushless);
        jointMotor = new CANSparkMax(armJoint.rotMotorID, MotorType.kBrushless);
    }
   // public Command ArmMoveCommand(){
        // Implement Arm code right here Jouji

    //}
}
