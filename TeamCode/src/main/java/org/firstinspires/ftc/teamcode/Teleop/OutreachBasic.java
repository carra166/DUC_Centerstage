package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class OutreachBasic extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double tgtPowerLeft = 0;
    double tgtPowerRight = 0;

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

            /*this cuts the target power in half if you're
            holding down the b button on the controller*/
            if (!this.gamepad1.b) {
                tgtPowerLeft = this.gamepad1.left_stick_y;
                tgtPowerRight = this.gamepad1.right_stick_y;
            } else {
                tgtPowerLeft = (this.gamepad1.left_stick_y / 2);
                tgtPowerRight = (this.gamepad1.right_stick_y / 2);
            }

            //sets motor power
            frontLeft.setPower(tgtPowerLeft);
            backLeft.setPower(tgtPowerLeft);
            frontRight.setPower(-tgtPowerRight);
            backRight.setPower(-tgtPowerRight);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}