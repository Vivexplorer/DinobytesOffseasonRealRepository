package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "ArmExtensionPower")
public class ArmExtensionPower extends OpMode {

    private DcMotorEx armExtender;
    @Override
    public void init() {
        armExtender = hardwareMap.get(DcMotorEx.class, "armExtender");
    }
    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            armExtender.setPower(0.5);
        }
    }
}
