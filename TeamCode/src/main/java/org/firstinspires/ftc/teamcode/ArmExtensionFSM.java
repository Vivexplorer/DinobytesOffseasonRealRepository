package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class ArmExtensionFSM{
    PIDController pidController = new PIDController(kp,ki,kd);
    public static double kp, kd, ki, kf, target;
    public DcMotor ArmExtension;


    public ArmExtensionFSM (HardwareMap hardwareMap) {
        ArmExtension = hardwareMap.get(DcMotor.class, "LeftMotorPivot");
    }

    public void setTarget(double newTarget){
        this.target = newTarget;
    }
    public void update(){
        pidController.setPID(kp, ki, kd);
        int armPos = ArmExtension.getCurrentPosition();
        double pid = pidController.calculate(armPos, target);

        double power = pid;

        ArmExtension.setPower(power);
    }
}