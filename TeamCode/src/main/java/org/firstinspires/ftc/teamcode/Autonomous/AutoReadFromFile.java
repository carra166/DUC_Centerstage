package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Environment;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.processors.ducProcessorBlueLeft;
import org.firstinspires.ftc.vision.VisionPortal;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/*
10430 AUTONOMOUS PROGRAM
Uses encoders
*/
@Autonomous
@Config
public class AutoReadFromFile<myIMUparameters> extends LinearOpMode {

    int step = 0;
    boolean runOnce = true;

    private static final String BASE_FOLDER_NAME = "autonomousTexts";
    static String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
    public static String textFileName = "test";
    public static double speed = 0.1;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor armMotor;
    private DcMotor armMotor2;
    Servo servoWrist;
    CRServo servoIntake;

    private IMU imu;
    private YawPitchRollAngles robotOrientation;
    private ducProcessorBlueLeft ducProcessor;
    private double duckPosition;

    InterpLUT armAngles;
    double kCos = 0;
    double tgtArmPower = 0;
    double armPosition = -20;

    double kp = 0.005;
    String highestPosition = "";

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");
        initializeMotors();

        armAngles = new InterpLUT();
        armAngles.add(-100, -41); //safety 1
        armAngles.add(0, -40); //init position
        armAngles.add(100, 0); //straight out (forward)
        armAngles.add(400, 90); //straight up
        armAngles.add(666, 180); //straight back
        armAngles.add(1000, 181); //safety 2
        armAngles.createLUT();

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

        ducProcessor = new ducProcessorBlueLeft();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(ducProcessor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}
        servoWrist.setPosition(0);

        while (!isStarted() && !isStopRequested()) {
            duckPosition = ducProcessor.getDuckPosition();
            telemetry.addData("DUCK POSITION", duckPosition);
            telemetry.update();
        }

        //start auto

        /*while (isStarted() && !isStopRequested()) {
            tgtArmPower = calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos, kp, armPosition);
            armMotor.setPower(tgtArmPower);
            armMotor2.setPower(-tgtArmPower);

            switch (step) {
                case 0:
                    if (runOnce) {

                        runOnce = false;
                    }
                    if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                        step++;
                        runOnce = true;
                    }
                break;
            }*/

        List<double[]> parsedLines = readAndParseDoublesFromFile();

        for (double[] line : parsedLines) {
            runToParsedPosition(line, 0.1);
        }

        //linearMovement(1.0, 1440, "Test");

        telemetry.addLine(parsedLines.get(0)[0]+"");
        telemetry.update();

    }

    private double inchesToEncoders(double inches) {
        return inches;
    }

    private void runToParsedPosition(double[] myLine, double power) {

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setTargetPosition((int)myLine[0]);
        frontLeft.setTargetPosition((int)myLine[1]);
        backRight.setTargetPosition((int)myLine[2]);
        backLeft.setTargetPosition((int)myLine[3]);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Drive(speed, myLine);

        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy()) {}

    }

    private void initializeMotors() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "ARM1");
        armMotor2 = hardwareMap.get(DcMotor.class, "ARM2");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoIntake = hardwareMap.get(CRServo.class,"intake");
        servoWrist = hardwareMap.get(Servo.class, "wrist");

    }

    private void linearMovement(double power, int distance, String type) {
        //POSITIVE VALUES GO FORWARD, NEGATIVE VALUES GO BACKWARD

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        switch (type) {
            case "backward":
                frontRight.setTargetPosition(-distance);
                frontLeft.setTargetPosition(distance);
                backRight.setTargetPosition(-distance);
                backLeft.setTargetPosition(distance);
            break;
        } //this takes the type input variable and sets the direction accordingly
        //for example, the "forward" input would make all motors go forward (obviously)

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Drive(power);

        while (frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            //telemetry.addData("Type of movement", type);
        }

        //sets power to zero, therefore braking
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(500); // Use this after setting power to 0 to give it time to brake

    }

    private void Drive(double power, double[] targetPositions) {

        double highestPositionIndex = findHighest(targetPositions);
        double highestPosition = targetPositions[(int)highestPositionIndex];
        double[] powerPercentagesArray = new double[]{ (Math.abs(targetPositions[0]/highestPosition) * 100), (Math.abs(targetPositions[1]/highestPosition) * 100), (Math.abs(targetPositions[2]/highestPosition) * 100), (Math.abs(targetPositions[3]/highestPosition) * 100) };
        DcMotor[] motors = new DcMotor[]{ frontRight, frontLeft, backRight, backLeft };

        for (int i = 0; i<4; i++) {
            if (targetPositions[i] > 0) {
                motors[i].setPower(power*(powerPercentagesArray[i]/100));
            } else {
                motors[i].setPower(-power*(powerPercentagesArray[i]/100));
            }
        }
    }

    private double calculateArmPower(double armAngle, double kCos, double kp, double target) {
        return kCos * Math.cos(Math.toRadians(armAngle)) + (kp * (target-armAngle));
    }

    private static List<double[]> readAndParseDoublesFromFile() {
        List<double[]> parsedLines = new ArrayList<>();
        try {
            FileReader reader = new FileReader(directoryPath+"/"+textFileName+".txt");
            BufferedReader bufferedReader = new BufferedReader(reader);

            String line;
            while ((line = bufferedReader.readLine()) != null) {
                String[] numbersAsString = line.split(" ");
                double[] parsedNumbers = new double[numbersAsString.length];
                for (int i = 0; i < numbersAsString.length; i++) {
                    parsedNumbers[i] = Double.parseDouble(numbersAsString[i]);
                }
                parsedLines.add(parsedNumbers);
            }

            bufferedReader.close();
        } catch (IOException e) {
            System.out.println("An error occurred while reading doubles.");
            e.printStackTrace();
        }
        return parsedLines;
    }

    //i love chat-gpt!
    public static double findHighest(double[] arr) {
        double highest = arr[0];
        double highestI = 0;
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] > highest) {
                highest = arr[i];
                highestI = i;
            }
        }
        return highestI;
    }

}