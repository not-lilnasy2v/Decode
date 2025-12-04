package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class servoGasire  extends LinearOpMode {
    private ServoImplEx aruncare;
    double pos= 0.5;
    @Override
    public void runOpMode(){
        aruncare = hardwareMap.get(ServoImplEx.class, "sortare");
// 1 luare 0.263 2 luare 0.623 3 luare 1
        // 1 aruncare 0.803 3 aruncare 0.443 2 aruncare 0.062

        waitForStart();
        while (opModeIsActive()){


            if(gamepad1.a){
                pos += 0.0001;
            }
            if(gamepad1.b){
                pos -= 0.0001;
            }


            aruncare.setPosition(pos);
            telemetry.addData("pos", pos);
            telemetry.update();
        }

    }
}
