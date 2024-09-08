package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config// - for ftcdashboard
@TeleOp(name = "ArmPivotFSM")
public class ArmPivotFSM extends OpMode {
    PIDController pidController = new PIDController(kp,ki,kd);
    public static double kp = 0.005, kd = 0.0001, ki = 0, kf = 0.05, target =0;
    public static double ticksInInches = 720/180;// i tune this by extending my system to some place and dividing the read encoder ticks by the length(in your case degrees);
    public DcMotor LeftMotor;

    public DcMotor RightMotor;

    public DcMotorEx armExtender;



    public void setTarget(double newTarget){
        this.target = newTarget;
    }

    public ArmPivotFSM (HardwareMap hardwareMap) {
        LeftMotor = hardwareMap.get(DcMotor.class, "leftRobotArm");
        RightMotor = hardwareMap.get(DcMotor.class, "rightRobotArm");

        armExtender = hardwareMap.get(DcMotorEx.class, "armExtender");


        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        pidController.setPID(kp, ki, kd);
        int armPos = (RightMotor.getCurrentPosition());
        double pid = pidController.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticksInInches)) * kf;

        double power = pid + ff;

        RightMotor.setPower(power);
        LeftMotor.setPower(power);

        telemetry.addData("rightArmPos: ", RightMotor.getCurrentPosition());
        telemetry.update();

        if(gamepad1.right_bumper) {
            armExtender.setPower(0.8);
        } else if(gamepad1.left_bumper) {
            armExtender.setPower(-0.8);
        } else{
            armExtender.setPower(0);
        }
    }
    @Override
    public void init() {
        ArmPivotFSM armPivotFSM = new ArmPivotFSM(hardwareMap);
    }
    @Override
    public void loop() {
        update();

    }
}
