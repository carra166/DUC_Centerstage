package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleopMK1 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double tgtPower = 0;

    @Override
    public void runOpMode() {

        //initialized motors
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (!this.gamepad1.b) {
                tgtPower = this.gamepad1.left_stick_y;
            } else {
                tgtPower = (this.gamepad1.left_stick_y / 2);
            }

            frontLeft.setPower(tgtPower);
            backLeft.setPower(tgtPower);
            frontRight.setPower(-tgtPower);
            backRight.setPower(-tgtPower);

            telemetry.addData("Is b button pressed?", this.gamepad1.b);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}