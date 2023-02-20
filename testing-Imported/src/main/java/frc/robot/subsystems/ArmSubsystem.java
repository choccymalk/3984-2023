package frc.robot.subsystems;
import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.Swerve.arm;


public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax shoulderMotor; 
    private CANSparkMax jointMotor;
    public RelativeEncoder EncoderShoulder;
    public RelativeEncoder EncoderJoint;
    public ArmFeedforward ShoulderFF;
    public ArmFeedforward JointFF;
    public SparkMaxPIDController ShoulderPID;
    public SparkMaxPIDController JointPID;

    // Initialize the goal point for the arm.
    private double[] goal = new double[2];

    public ArmSubsystem() {
        ShoulderFF = new ArmFeedforward(
            arm.Shoulder.Ks, 
            arm.Shoulder.Kg, 
            arm.Shoulder.Kv
        );
        JointFF = new ArmFeedforward(
            arm.Joint.Ks,
            arm.Joint.Kg, 
            arm.Joint.Kv
        );
        shoulderMotor = new CANSparkMax(
            arm.Shoulder.rotMotorID, 
            MotorType.kBrushless
        );
        jointMotor = new CANSparkMax(
            arm.Joint.rotMotorID, 
            MotorType.kBrushless
        );

        EncoderShoulder = shoulderMotor.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        EncoderShoulder.setPosition(0);
        EncoderShoulder.setPositionConversionFactor(
            360/arm.Shoulder.gearRatio
        );
        EncoderShoulder.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0
        );
        EncoderJoint = jointMotor.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        EncoderJoint.setPosition(0);
        EncoderJoint.setPositionConversionFactor(
            360/arm.Joint.gearRatio
        );
        EncoderShoulder.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0
        );

        ShoulderPID = shoulderMotor.getPIDController();
        ShoulderPID.setP(arm.Shoulder.Kp);
        ShoulderPID.setI(arm.Shoulder.Ki);
        ShoulderPID.setD(arm.Shoulder.Kd);

        JointPID = jointMotor.getPIDController();
        JointPID.setP(arm.Joint.Kp);
        JointPID.setI(arm.Joint.Ki);
        JointPID.setD(arm.Joint.Kd);

    }
    public double[] getPositions(){
         // index 0 is shoulderPos, index 1 is jointPos
        double[] poss = new double[]{EncoderShoulder.getPosition(), EncoderJoint.getPosition()};
        return poss;
    }

    public double[] getErrors(double targetJoint, double targetShoulder){
        double[] errors = new double[2];
        double errorJoint = targetJoint - getPositions()[1];
        double errorShoulder = targetShoulder - getPositions()[0];
        errors[0] = errorShoulder;
        errors[1] = errorJoint;
        return errors;
    }
    public void runArm(double speedShoulder, double speedJoint){
        shoulderMotor.set(speedShoulder);
        jointMotor.set(speedJoint);
    }
    public void stopArm(){
        shoulderMotor.set(0);
        jointMotor.set(0);
    }
    public void moveToAngle(double angle){
        ShoulderPID.setReference(
            angle,
            ControlType.kPosition,
            0,
            ShoulderFF.calculate(
                (angle/360) * 2 * Math.PI, 
                0
            )
        );
        JointPID.setReference(
            angle,
            ControlType.kPosition, 
            0, 
            JointFF.calculate(
                (angle/360) * 2 * Math.PI, 
                0
            )
        );

    }
    public boolean atPosShoulder(double angle){
        return (Math.abs(getErrors(0, angle)[0]) < 1);
    }
    public boolean atPosJoint(double angle){
        return (Math.abs(getErrors(angle, 0)[1]) < 1);
    }


    public Rotation2d[] getAngles(double x, double y){
        double AngleShoulder = 0;
        double AngleJoint = 0;
        Rotation2d[] angles = new Rotation2d[2];
        //Implement get angle code here 
        double arm1Length = 34;
        
		double jointLength = 23.5;
		double hypotenuse = Math.hypot(x, y);

		AngleJoint = Math.pow(hypotenuse,  2) - Math.pow(arm1Length, 2) - Math.pow(jointLength, 2);

		AngleJoint = AngleJoint/(-2*arm1Length*jointLength);

		AngleJoint = Math.acos(AngleJoint);
		
        
        double theta2 = Math.pow(jointLength,2)-Math.pow(arm1Length,2)-Math.pow(hypotenuse,2);

        theta2 = theta2/(-2*arm1Length*hypotenuse);

        double j = Math.atan(0/23);

        double k = theta2-j;

        AngleShoulder = ((Math.PI/2)-k);
        
        //convert angle to radians
        angles[0] = new Rotation2d(AngleShoulder);
        angles[1] = new Rotation2d(AngleJoint);
        return angles;
    }
    /*************************************************/

    public Command moveTo(double x, double y){
        double degrees = getAngles(x, y)[0].getDegrees();
        return run(() -> {
            moveToAngle(degrees);
        }).until(() -> (atPosShoulder(degrees) /*&& atPosJoint(degrees)*/));
    }

    public void periodic(){
        double joi = EncoderJoint.getPosition() * 360;
        double sho = EncoderShoulder.getPosition() * 360;
        SmartDashboard.putNumber("Shoulder", sho);
        SmartDashboard.putNumber("Joint", joi);
        Rotation2d[] angles = this.getAngles(goal[0], goal[1]);
        double angle = angles[0].getDegrees();
        SmartDashboard.putNumber("ShoulderGoal ", angle);
        angle = angles[1].getDegrees();
        SmartDashboard.putNumber("JointGoal ", angle);

    }
    /*public void getNewPoint(){
        goal[0] = goal[0] + 0.05;
        goal[1] = goal[0] + 0.05;
    }*/
}
