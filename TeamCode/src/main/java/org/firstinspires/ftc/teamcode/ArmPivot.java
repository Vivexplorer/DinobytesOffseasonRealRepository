package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config// - for ftcdashboard
@TeleOp(name = "ArmPivot")
public class ArmPivot extends OpMode {
    PIDController pidController = new PIDController(kp,ki,kd);
    public static double kp = 0, kd = 0, ki = 0, kf = 0, target =0;
    public static double ticksInInches = 720/180;// i tune this by extending my system to some place and dividing the read encoder ticks by the length(in your case degrees);
    public DcMotor LeftMotor;

    public DcMotor RightMotor;

    public ArmPivot (HardwareMap hardwareMap) {
        LeftMotor = hardwareMap.get(DcMotor.class, "leftRobotArm");
        RightMotor = hardwareMap.get(DcMotor.class, "rightRobotArm");
    }

    public void setTarget(double newTarget){
        this.target = newTarget;
    }
    public void update(){
        pidController.setPID(kp, ki, kd);
        int armPos = (RightMotor.getCurrentPosition()) / 2;
        double pid = pidController.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticksInInches)) * kf;

        double power = pid + ff;

        RightMotor.setPower(power);
        LeftMotor.setPower(power);
    }
    @Override
    public void init() {
    }
    @Override
    public void loop() {
        update();
    }
}
