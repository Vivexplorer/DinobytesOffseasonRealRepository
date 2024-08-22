package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config// - for ftcdashboard
public class ArmPivot{
    PIDController pidController = new PIDController(kp,ki,kd);
    public static double kp, kd, ki, kf, target;
    public static double ticksInInches = 720/180;// i tune this by extending my system to some place and dividing the read encoder ticks by the length(in your case degrees);
    public DcMotor LeftMotor;

    public DcMotor RightMotor;

    public ArmPivot (HardwareMap hardwareMap) {
        LeftMotor = hardwareMap.get(DcMotor.class, "LeftMotorPivot");
        RightMotor = hardwareMap.get(DcMotor.class, "RightMotorPivot");
    }

    public void setTarget(double newTarget){
        this.target = newTarget;
    }
    public void update(){
        pidController.setPID(kp, ki, kd);
        int armPos = (RightMotor.getCurrentPosition() + LeftMotor.getCurrentPosition()) / 2;
        double pid = pidController.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticksInInches)) * kf;

        double power = pid + ff;

        RightMotor.setPower(power);
        LeftMotor.setPower(power);
    }
}
