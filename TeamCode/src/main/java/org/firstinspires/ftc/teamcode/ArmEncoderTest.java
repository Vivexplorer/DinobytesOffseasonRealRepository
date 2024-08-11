package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class ArmEncoderTest extends OpMode {

    private DcMotorEx LeftArmPivot;
    private DcMotorEx RightArmPivot;
    @Override
    public void init() {
        LeftArmPivot = hardwareMap.get(DcMotorEx.class, "leftRobotArm");
        RightArmPivot = hardwareMap.get(DcMotorEx.class, "rightRobotArm");
    }
    @Override
    public void loop() {
        telemetry.addData("LeftArmPivot ", LeftArmPivot.getCurrentPosition());
        telemetry.addData("RightArmPivot ", RightArmPivot.getCurrentPosition());
        telemetry.update();
    }
}
