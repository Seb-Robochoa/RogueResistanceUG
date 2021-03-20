package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.robotcore.external.function.InterruptableThrowingRunnable;
import org.firstinspires.ftc.robotcore.external.function.InterruptableThrowingSupplier;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.hardware.bosch.BNO055IMU;

import java.util.Timer;
//import gayness
@Autonomous
public class AutoMain extends LinearOpMode {
    private static double forwardAngle;
    private final int[] RIGHT = {1, 2, 2, 1},
            LEFT = {2, 1, 1, 2},
            FORWARD = {1, 1, 1, 1},
            BACKWARD = {2, 2, 2, 2},
            FOWARDRIGHT = {1, 0, 0, 1},
            FORWARDLEFT = {0, 1, 1, 0},
            BACKWARDRIGHT = {0, 2, 2, 0},
            BACKWARDLEFT = {2, 0, 0, 2},
            TURNRIGHT = {1, 2, 1, 2},
            TURNLEFT = {2, 1, 2, 1};
    private DcMotorEx leftFront, rightFront, leftBack, rightBack, arm, transfer, intake, shooter;
    private Servo claw, flicker, holder;
    private DcMotorEx[] motors;
    private final double TPI = 33.5625;
    private int scenario;
    OpenCvInternalCamera phoneCam;
    BNO055IMU imu;
    AutoMain.UltimateGoalDeterminationPipeline pipeline;
    private int distance;
    private int x;
    private int y;
    Orientation angles;
    private String currentDirection;
    Orientation ang;


    @Override
    public void runOpMode() throws InterruptedException { //Load Zone B
        initialize();
        //powerShot();



        moveBot(FORWARD,60, .75, true); //forward
        pause();
        updateY(60);
        moveBot(RIGHT,24, .75, true); //strafe right
        pause();
        updateX(20);
        moveBot(TURNRIGHT, 1, .75, true); //turn
        pause();
        holder.setPosition(0);
        snapBot();
        yeetRing(-.73);
        //secondWobbler();
        //------------

        int deltaY = getY() - 43;
        int deltaX = 18 - getX();

        //coordinates for shooting
        int targetX = getX();
        int targetY = getY();

        moveBot(TURNRIGHT, 1, .75, true); //turn
        pause();

        moveBot(RIGHT, deltaX, .75, true); //right
        pause();
        moveBot(BACKWARD, deltaY, .5, true); //backward
        pause();
        updateY(deltaY);
        // moveBot(1, 2, 2, 1, deltaX, .6, true); //right
        updateX(deltaX);
        intake(.1);

        moveBot(RIGHT, targetX-getX(), .75, true);
        pause();
        updateX(targetX-getX());
        int save = targetX-getX();
        //snapBot();
        yeetRing(-.73);


        moveBot(LEFT, -save, .75, true); //right
        pause();
        updateX(save);
        intake(.2);
        moveBot(RIGHT, save, .75, true);
        pause();
        //snapBot();
        yeetRing(-.73);




        //moveBot(1,2,2,1,4, .6, true); //strafe right
        //updateX(24);

        // holder.setPosition(0);
        // moveBot(1,1,1,1,-12, .6, true); // backward
        //updateY(48);
        // intake();
        //moveBot(1,2,2,1,-4, .6, true); //strafe left
        //updateX(16);
        // moveBot(1,1,1,1,12, .6, true); // forward
        //updateY(60);
        updateTelemetry();
        /*if(scenario == 0) //zone A
        {
            moveBot(FORWARD, 15, .6, true); //forward
            updateY(15);

            moveBot(TURNRIGHT, 16, .6, true); //turn
            //x += -16;
            armTravel();
            updateTelemetry();
        }
        if(scenario == 1) { //zone B
            moveBot(FORWARD, 18, .6, true); //forward
            updateY(18);
            moveBot(RIGHT,18, .6, true); //strafe right
            updateX(18);
            armTravel();
            updateTelemetry();
        }
        if(scenario == 4){ //zone C
            moveBot(LEFT,11, .6, true); //strafe left
            updateX(-11);
            moveBot(TURNRIGHT,1,.6,true);
            moveBot(FORWARD, 48, .6, true); //forward
            updateY(48);
            armTravel();
            moveBot(BACKWARD, 36, .6, true); //backward
            updateY(-36);
            updateTelemetry();
        }


        updateTelemetry();
        // Target position is x:7 y: 60

        // int currentX = getX();
        //int currentY = getY();

        /*
        int currentY = 60 - getY();
        int currentX = 20 - getX();
        moveBot(1, 1, 1, 1, currentY, .6, true); //forward
        moveBot(1,2,2,1, currentX, .6, true); //strafe left
        moveBot(2, 1, 2, 1, -5, .6, true); //turn
        moveBot(1, 1, 1, 1, -15, .6, true); //forward
        */

        claw.setPosition(1);
        turn(180);
        turn(90);
        turn(-90);
        turn(-180);

        //0 Void 1 Forward 2 Reverse
        //moveBot(1,1,2,3,18, .6, true);'
        //if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.NONE)
        // moveBot(1,1,2,3,12, .6, true);
        // else if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.ONE)
        // moveBot(1,1,2,3,24, .6, true);
        // else if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.FOUR)
        // moveBot(1,1,2,3,48, .6, true);
        //moveBot(1, 1, 2, 2, 48, .60, true); //forward
        //f();
        //moveBot(1, 1, 2, 2, 12, .60, true); //forward
        //setDownWobbler();
        //activate flicker


    }

    public void secondWobbler() throws InterruptedException {
        // position of the second wobbler is at 24, 5
        int deltaX = getX();
        int deltaY = getY() - 5;
        turn(180);
        moveBot(FORWARD, deltaY-7, .4, true);

        ElapsedTime wobbler = new ElapsedTime();
        arm.setTargetPosition(-2268);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (wobbler.milliseconds() <= 3000) {
            heartbeat();
            if (!arm.isBusy()) {
                claw.setPosition(0);
            }
        }
        claw.setPosition(1);
        arm.setTargetPosition(2268);

    }



    public void updateTelemetry() {
        telemetry.addData("X", getX());
        telemetry.addData("Y", getY());
        telemetry.update();
    }

    public void updateX(int pos) {
        x += pos;
    }

    public void updateY(int pos) {
        y += pos;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public void initialize() {
        //Even if I'm not using it, I have to map it because it is mapped on the bot.
        x = 0;
        y = 0;
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFrontDrive");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftRearDrive");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFrontDrive");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightRearDrive");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        transfer = (DcMotorEx) hardwareMap.dcMotor.get("transfer");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        claw = hardwareMap.servo.get("claw");
        flicker = hardwareMap.servo.get("flicker");
        holder = hardwareMap.servo.get("holder");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new AutoMain.UltimateGoalDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        claw.setPosition(1);
        flicker.setPosition(.7);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        forwardAngle = angles.firstAngle;
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(() -> {
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        });
        while (!opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            updateTelemetry();
            telemetry.update();
            scenario = 1;
            if (pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.NONE) //zone A
            {
                scenario = 0;
            }
            if (pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.ONE) //zone A
            {
                scenario = 1;
            }
            if (pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.FOUR) //zone A
            {
                scenario = 4;
            }
            sleep(50);
        }
        motors = new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack};
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (motor == leftFront || motor == leftBack)
                motor.setDirection(DcMotor.Direction.REVERSE);
            else
                motor.setDirection(DcMotor.Direction.FORWARD);
        }
        waitForStart();
    }
//from retard import idiot

    public void snapBot() throws InterruptedException{
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        turn(forwardAngle-angles.firstAngle);
    }
    /**
     * @param power
     * @param distance inches so you will have to convert to tics
     */

    //This method accepts 6 variables:
    // 4 of them are motors (leftFront, leftBack, rightFront, rightBack)
    //These variables will accept 3 integers: 0 Void, 1 Forward, 2 Reverse
    //It also accepts power and distance

    //withIntake will be used later (work in progress)
    public void betterMoveBot(double angleRadians, double power, int distance, double turnAmount, boolean withIntake) throws InterruptedException { //turnAmount is + or - depending on right or left, in range -1 to 1
        //testing needed for how much distance = 90 degrees
        //estimated total x + total y movement distance needed in distance variable for angled slides (needs testing)
        double powerAngle = angleRadians - (Math.PI / 4); // conversion for correct power values
        double[] motorPowers = new double[4];//left front, left back, right front, right back
        motorPowers[0] = Range.clip(0.5 * Math.cos(powerAngle) - turnAmount, -1.0, 1.0);
        motorPowers[1] = Range.clip(0.5 * Math.sin(powerAngle) - turnAmount, -1.0, 1.0);
        motorPowers[2] = Range.clip(0.5 * Math.sin(powerAngle) + turnAmount, -1.0, 1.0);
        motorPowers[3] = Range.clip(0.5 * Math.cos(powerAngle) + turnAmount, -1.0, 1.0);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        int travel = (int) (distance * TPI);
        for (int i = 0; i < 4; i++) {
            motors[i].setTargetPosition(travel);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setPower(motorPowers[i] * (power / 2));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            heartbeat();
        }
    }

    public void moveBot(int[] dir, int distance, double power, boolean withIntake) throws InterruptedException {
        //moveBot(1, 1, 2, 2, -24, .60, true); //Forwward
        //turnBot(2, 2, 1, 1, -24, .60); //Backward
        //turnBot(2, 1, 2, 1, 30, .60); //Strafe right
        //turnBot(1, 2, 1, 2, 0, .6) /Strafe left
        int leftT = dir[0];
        int rightT = dir[1];
        int leftB = dir[2];
        int rightB = dir[3]; // MAKE CASE FOR 0
        if (leftT == 1) {
            leftFront.setDirection(DcMotor.Direction.FORWARD);
        } else if (leftT == 2) {
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        }

        if (leftB == 1) {
            leftBack.setDirection(DcMotor.Direction.FORWARD);
        } else if (leftB == 2) {
            leftBack.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightT == 2) {
            rightFront.setDirection(DcMotor.Direction.FORWARD);
        } else if (rightT == 1) {
            rightFront.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightB == 2) {
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        } else if (rightT == 1) {
            rightBack.setDirection(DcMotor.Direction.REVERSE);
        }

        /*if(withIntake) {
            while (shooterTime.milliseconds() <= 5000) {
                intake.setPower(.5);
                transfer.setPower(1);
                heartbeat();
            }
        }*/

        //Moves the robot
        int travel = (int) (distance * TPI);
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(travel);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //This is what checks if the motors are supposed to be still running.
        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            heartbeat();
        }
        //intake.setPower(0);
        // transfer.setPower(0);
    }

    public void powerShot() throws InterruptedException {
        double pow = -.69;
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(pow);
        //Target position for shooting
        double targetX = 26.5;
        double targetY = 50.5;
        double initialDistance = 60;
        double powerIncrement = 0.04;
        ElapsedTime wait = new ElapsedTime();

        while (wait.milliseconds() <= 1000) {
        }
        yeetLessRing(pow);
        turn(Math.toDegrees(Math.atan2(7.5, initialDistance)));
        yeetLessRing(pow-powerIncrement);
        turn(Math.toDegrees(Math.atan2(15,initialDistance))-Math.toDegrees(Math.atan2(7.5,initialDistance)));
        yeetLessRing(pow-(2*powerIncrement));
        shooter.setPower(0);
    }

    public void pause(){
        ElapsedTime bruh = new ElapsedTime();
        while(bruh.milliseconds()<100){};
    }

    public void intake(double pow) throws InterruptedException {
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition((int) (TPI * -6));
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(pow);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(-1);
            transfer.setPower(1);
        }
        ElapsedTime succ = new ElapsedTime();
        //while(succ.milliseconds() < 3000)
        //  heartbeat();
        // intake.setPower(-1);
        //transfer.setPower(1);
        while (succ.milliseconds() < 5000)
            heartbeat();
        intake.setPower(0);
        transfer.setPower(0);
    }

    public void yeetRing(double pow) throws InterruptedException {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(pow - .05);
        ElapsedTime shooterTime = new ElapsedTime();
        int flick = 2000;
        while (shooterTime.milliseconds() <= 5000) {
            flick = (int) shooterTime.milliseconds() + flick;
            if (shooterTime.milliseconds() >= 2000) {
                while (shooterTime.milliseconds() <= flick)
                    flicker.setPosition(0);
                shooter.setPower(pow);
            }
            flick = (int) shooterTime.milliseconds() + 500;
            while (shooterTime.milliseconds() <= flick)
                flicker.setPosition(.7);
            flick = 500;

        }
        flicker.setPosition(.7);
        intake.setPower(0);
        transfer.setPower(0);
        shooter.setPower(0);
    }

    public void yeetLessRing(double pow) throws InterruptedException {
        shooter.setPower(pow);
        ElapsedTime shooterTime = new ElapsedTime();
        while (shooterTime.milliseconds() <= 500) {
            if (shooterTime.milliseconds() <= 250) flicker.setPosition(0);
            else flicker.setPosition(.7);
        }
        while (shooterTime.milliseconds() <= 1000) {
        }
    }
//julian said make a variable called balls someone please do that

    public void heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
    }

    public void armTravel() throws InterruptedException {
        ElapsedTime wobbler = new ElapsedTime();
        arm.setTargetPosition(-2268);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (wobbler.milliseconds() <= 3000) {
            heartbeat();
            if (!arm.isBusy()) {
                claw.setPosition(0);
            }
        }
        arm.setTargetPosition(2268);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy()) {
            heartbeat();
        }
        arm.setPower(0);

    }

    public void turn(double turnAmount) throws InterruptedException {//right is positive sdlijhfkjdhfjklashdflkasdhjklfahsdjklfhasjkldfhlasjdkhfjkasfdhlk
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;
        double desiredAngle = startAngle+turnAmount;
        int turnFactor = (int) (turnAmount/Math.abs(turnAmount));// (turnAmount/Math.abs(turnAmount) determines if right or left. if left this value will be -1 and swap power values
        double initialPower = /*Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))>30?.5:*/Range.clip(.5/(60/Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))),.05,.5);//.5/(30/x) x is how close to desired angle

        telemetry.addData("desired angle change", turnAmount);
        telemetry.addData("current angle", angles.firstAngle);
        telemetry.addData("start angle", startAngle);
        telemetry.addData("degrees turned", Math.abs(angles.firstAngle-startAngle));
        telemetry.addData("desired angle", desiredAngle);
        telemetry.addData("angle difference", Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount)));
        telemetry.addData("initial power", initialPower);
        telemetry.addData("current power", Range.clip(.5/(60/Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))),.1,.5));
        telemetry.update();
        leftFront.setPower(initialPower*turnFactor);
        rightFront.setPower(-initialPower*turnFactor);
        leftBack.setPower(initialPower*turnFactor);
        rightBack.setPower(-initialPower*turnFactor);

        while((int)Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))>5){ // (angles.firstAngle-startAngle-Math.abs(turnAmount)) is the difference between current angle and desired. Closer to desired angle = lower value.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //if(Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))<30){
            leftFront.setPower(Range.clip(.5/(60/Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))),.1,.5)*turnFactor);
            rightFront.setPower(-Range.clip(.5/(60/Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))),.1,.5)*turnFactor);
            leftBack.setPower(Range.clip(.5/(60/Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))),.1,.5)*turnFactor);
            rightBack.setPower(-Range.clip(.5/(60/Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))),.1,.5)*turnFactor);
            //}
            telemetry.addData("desired angle change", turnAmount);
            telemetry.addData("current angle", angles.firstAngle);
            telemetry.addData("start angle", startAngle);
            telemetry.addData("degrees turned", Math.abs(angles.firstAngle-startAngle));
            telemetry.addData("desired angle", desiredAngle);
            telemetry.addData("angle difference", Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount)));
            telemetry.addData("initial power", initialPower);
            telemetry.addData("current power", Range.clip(.5/(60/Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))),.1,.5));
            telemetry.update();
            heartbeat();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        ElapsedTime wait = new ElapsedTime();
        while(wait.milliseconds()<1000){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("desired angle change", turnAmount);
            telemetry.addData("current angle", angles.firstAngle);
            telemetry.addData("start angle", startAngle);
            telemetry.addData("degrees turned", Math.abs(angles.firstAngle-startAngle));
            telemetry.addData("desired angle", desiredAngle);
            telemetry.addData("angle difference", Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount)));
            telemetry.addData("initial power", initialPower);
            telemetry.addData("current power", Range.clip(.5/(60/Math.abs(Math.abs(angles.firstAngle-startAngle)-Math.abs(turnAmount))),.1,.5));
            telemetry.update();
            heartbeat();
        }
    }

    public void turn(int degree, String dir)throws InterruptedException{
        //left to pos
        //right to neg
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int deg = 0;
        if (dir.equals("right"))
            deg = -degree;
        else if (dir.equals("left"))
            deg = degree;
        telemetry.addData("current angle", angles.firstAngle);
        telemetry.addData("desired angle", deg);
        telemetry.update();
        if (dir.equals("left")) {
            while ((int)angles.firstAngle < deg) {

                if ((int)angles.firstAngle < deg-30) {
                    leftFront.setPower(.5);
                    leftBack.setPower(.5);
                    rightFront.setPower(-.5);
                    rightBack.setPower(-.5);
                }
                else {
                    leftFront.setPower(.05);
                    leftBack.setPower(.05);
                    rightFront.setPower(-.05);
                    rightBack.setPower(-.05);
                }

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.addData("angle", angles.secondAngle);
                telemetry.addData("angle", angles.thirdAngle);
                telemetry.update();
                heartbeat();
            }
        }
        else if (dir.equals("right")) {
            while ((int)angles.firstAngle > deg) {
                if ((int)angles.firstAngle > deg+30) {
                    leftFront.setPower(-.5);
                    leftBack.setPower(-.5);
                    rightFront.setPower(.5);
                    rightBack.setPower(.5);
                }
                else {
                    leftFront.setPower(-.05);
                    leftBack.setPower(-.05);
                    rightFront.setPower(.05);
                    rightBack.setPower(.05);
                }

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.addData("angle", angles.secondAngle);
                telemetry.addData("angle", angles.thirdAngle);
                telemetry.update();
                heartbeat();
            }
        }

    }






    public static class UltimateGoalDeterminationPipeline extends OpenCvPipeline {

        public enum RingPosition{
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(90, 80);

        static final int REGION_WIDTH = 10;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);

        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        private volatile RingPosition position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;

        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame){
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            if(avg1 > FOUR_RING_THRESHOLD){
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            } else if(avg1 > ONE_RING_THRESHOLD){
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.ONE;
            } else{
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
        public RingPosition getPosition() {
            return position;
        }


    }

    public static double getForwardAngle(){
        return forwardAngle;
    }


}