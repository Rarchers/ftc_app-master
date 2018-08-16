package teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Color;

/**
 * Created by Administrator on 2018/3/10.
 */

public class Rarcher_robot {

    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();

    public DcMotor MotorLF, MotorLB, MotorRF, MotorRB;
    public DcMotor MotorOpen, MotorLift;//open 是剪叉
    public Servo lfly, rfly, lflyup, rflyup,lfly2,rfly2,blup,brup,tight;
    private double vMax;
    private double vLF, vLB, vRF, vRB;
    private double lightValue = 1;

    public Rarcher_robot() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        lfly = hwMap.servo.get("lfly");
        rfly = hwMap.servo.get("rfly");
        lflyup = hwMap.servo.get("lflyup");
        rflyup = hwMap.servo.get("rflyup");
        lfly2 = hwMap.servo.get("lfly2");
        rfly2 = hwMap.servo.get("rfly2");
        blup = hwMap.servo.get("blup");
        brup = hwMap.servo.get("brup");
        tight = hwMap.servo.get("tight");
        MotorLF = hwMap.dcMotor.get("MotorLF");
        MotorLB = hwMap.dcMotor.get("MotorLB");
        MotorRF = hwMap.dcMotor.get("MotorRF");
        MotorRB = hwMap.dcMotor.get("MotorRB");

        MotorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorLift = hwMap.dcMotor.get("MotorFL2");
        MotorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorOpen = hwMap.dcMotor.get("MotorUp");

        MotorOpen.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //TODO.need fix !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //TODO.range 0 from 1 change setposition
        lfly.setPosition(0.5);
        rfly.setPosition(0.5);
        lflyup.setPosition(1);
        rflyup.setPosition(0);
        lfly2.setPosition(0.5);
        rfly2.setPosition(0.5);
        blup.setPosition(1);
        brup.setPosition(0);
        tight.setPosition(0);

        MotorLF.setDirection(DcMotor.Direction.REVERSE);
        MotorLB.setDirection(DcMotor.Direction.FORWARD);
        MotorRF.setDirection(DcMotor.Direction.FORWARD);
        MotorRB.setDirection(DcMotor.Direction.REVERSE);

        MotorOpen.setDirection(DcMotor.Direction.FORWARD);
        MotorLift.setDirection(DcMotor.Direction.FORWARD);

        MotorLF.setPower(0);
        MotorLB.setPower(0);
        MotorRF.setPower(0);
        MotorRB.setPower(0);

        MotorLift.setPower(0);

        MotorOpen.setPower(0);



        MotorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorOpen.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Stop() {
        MotorLB.setPower(0);
        MotorLF.setPower(0);
        MotorRB.setPower(0);
        MotorRF.setPower(0);
    }
//旋转  spain
    public void Run(double spin, double forward, double horizontal) {
        vLF = -spin + forward - horizontal;
        vLB = -spin + forward + horizontal;
        vRF = spin + forward + horizontal;
        vRB = spin + forward - horizontal;

        vMax = Math.max(Math.max(Math.abs(vLB), Math.abs(vLF)), Math.max(Math.abs(vRF), Math.abs(vRB)));
        if (vMax > 1) {
            vLB /= vMax;
            vLF /= vMax;
            vRB /= vMax;
            vRF /= vMax;
        }

        MotorLB.setPower(vLB);
        MotorLF.setPower(vLF);
        MotorRB.setPower(vRB);
        MotorRF.setPower(vRF);
    }

    public void Move(double spin, double forward, double horizontal, long periodMs) {
        Run(spin, forward, horizontal);
        waitForTick(periodMs);
        Stop();
    }


    public void waitForTick(long periodMs) {
        long remaining = periodMs - (long) period.milliseconds();
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        period.reset();
    }


}





