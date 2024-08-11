package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class armPivot extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    public final double ticks_in_degree = 730/180;

    private DcMotorEx LeftArmPivot;
    private DcMotorEx RightArmPivot;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LeftArmPivot = hardwareMap.get(DcMotorEx.class, "leftRobotArm");
        RightArmPivot = hardwareMap.get(DcMotorEx.class, "rightRobotArm");

        RightArmPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftArmPivot.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int armPos = RightArmPivot.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) *f;

        double power = pid +ff;

        LeftArmPivot.setPower(power);
        RightArmPivot.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target  ", target);
        telemetry.update();



    }
}
