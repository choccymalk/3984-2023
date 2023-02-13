package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.armJoint;
import frc.robot.Constants.Swerve.armShoulder;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax shoulderMotor; 
    private CANSparkMax jointMotor;
    private RelativeEncoder EncoderShoulder;
    private RelativeEncoder EncoderJoint;

    // Initialize the goal point for the arm.
    private double[] goal = new double[2];
    private double currAngle = 0;
    
    public ArmSubsystem() {
        shoulderMotor = new CANSparkMax(armShoulder.rotMotorID, MotorType.kBrushless);
        jointMotor = new CANSparkMax(armJoint.rotMotorID, MotorType.kBrushless);
        EncoderShoulder = shoulderMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        EncoderJoint = jointMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    }
    public void moveToAngle(){
        Rotation2d[] angles = new Rotation2d[2];
        //get angles needed for each joint
        angles = this.getAngles(goal[0], goal[1]);
        
        PIDController goToAngleShoulder = new PIDController(Constants.Swerve.sarmKP, Constants.Swerve.sarmKI, Constants.Swerve.sarmKD);
        PIDController goToAngleJoint = new PIDController(Constants.Swerve.jarmKP, Constants.Swerve.jarmKI, Constants.Swerve.jarmKD);

        double angle = angles[0].getRadians();
        goToAngleShoulder.setSetpoint(angle);
        goToAngleShoulder.setTolerance(0.05, 0.1);
        angle = angles[1].getRadians();
        goToAngleJoint.setSetpoint(angle);
        goToAngleJoint.setTolerance(0.05, 0.1);
        while (!goToAngleJoint.atSetpoint() && !goToAngleShoulder.atSetpoint()){
            shoulderMotor.setVoltage(goToAngleShoulder.calculate(EncoderShoulder.getPosition() * Math.PI - Constants.Swerve.armShoulder.angleOffset.getRadians())*.12);
            jointMotor.setVoltage(goToAngleJoint.calculate(EncoderJoint.getPosition()*Math.PI - Constants.Swerve.armShoulder.angleOffset.getRadians())*.12);
        }
    }
    /************************************************/
    public Rotation2d[] getAngles(double x, double y){
        int AngleShoulder = 0;
        int AngleJoint = 0;
        Rotation2d[] angles = new Rotation2d[2];
        //Implement get angle code here Jouji
        double arm1Length = 1;
        double arm2Length = 1;
        double hypotenuse = Math.hypot(x,y);
        AngleJoint = (int)Math.acos(-(Math.pow(hypotenuse,2)-(Math.pow(arm1Length,2)+Math.pow(arm2Length,2)))/(2*arm1Length+arm2Length));
        double theta2 = Math.acos(-(Math.pow(arm2Length,2)-(Math.pow(arm1Length,2)+Math.pow(hypotenuse,2)))/(2*arm1Length+hypotenuse));
        double j = Math.atan(y/x);
        double k = theta2-j;
        AngleShoulder = (int)(90-k);





        



        //convert angle to radians
        angles[1] = new Rotation2d(AngleShoulder);
        angles[2] = new Rotation2d(AngleJoint);
        return angles;
    }
    /*************************************************/

    public void setPoint(boolean intake, boolean low, boolean medium, boolean high){
        if (intake == true){
            goal[0] = Constants.Swerve.INTAKE[0];
            goal[1] = Constants.Swerve.INTAKE[1];
        }
        else if (low == true){
            goal[0] = Constants.Swerve.LOWGOAL[0];
            goal[1] = Constants.Swerve.LOWGOAL[1];
        }
        else if (medium == true){
            goal[0] = Constants.Swerve.MIDGOAL[0];
            goal[1] = Constants.Swerve.MIDGOAL[1];
        }
        else if (high == true){
            goal[0] = Constants.Swerve.HIGHGOAL[0];
            goal[1] = Constants.Swerve.HIGHGOAL[1];
        }
    }
    /*public void getNewPoint(){
        goal[0] = goal[0] + 0.05;
        goal[1] = goal[0] + 0.05;
    }*/
}
