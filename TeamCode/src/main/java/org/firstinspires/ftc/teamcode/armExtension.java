package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class armExtension {
    public DcMotorEx ArmExtension;

    public armExtension (HardwareMap hardwareMap) {
        ArmExtension = hardwareMap.get(DcMotorEx.class, "ArmExtension");
    }

    public class armUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            if (!initialized) {
                ArmExtension.setPower(1);
                initialized = true;
            }

            double pos = ArmExtension.getCurrentPosition();
            packet.put("armPos", pos);
            if (pos<780) {
                return true;
            } else {
                ArmExtension.setPower(0);
                return false;
            }

        }
    }

    public Action ArmUp() {
        return new armUp();
    }

}
