package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "drive")
public class LedOn extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        DcMotor ledPower = hardwareMap.get(DcMotor.class, "ledPower");
        waitForStart();
        while (!isStopRequested()) {
            ledPower.setPower(0.5);
        }

    }

}