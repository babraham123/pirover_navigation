
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <PID_v1.h>

double rInput = 0.0, rOutput = 0.0, lInput = 0.0, lOutput = 0.0;
double rSetpoint = 0.0;
double lSetpoint = 0.0;
PID rPID(&rInput, &rOutput, &rSetpoint, 10.0, 0.0, 2.0, DIRECT);
PID lPID(&lInput, &lOutput, &lSetpoint, 10.0, 0.0, 2.0, DIRECT);

const unsigned long serialPing = 500; // ping interval in ms
unsigned long lastMessage = 0;

#define rMotor 10
#define lMotor 11

#define rEncoder 2
#define lEncoder 3
volatile int rCounter = 0;
volatile int lCounter = 0;

void cmd_vel_r_cb(const std_msgs::Float32& cmd_msg)
{
    rSetpoint = (double)cmd_msg.data;
}

void cmd_vel_l_cb(const std_msgs::Float32& cmd_msg)
{
    lSetpoint = (double)cmd_msg.data;
}

ros::NodeHandle nh;

std_msgs::Int16 encoder_msg;
ros::Publisher pub_r("rwheel", &encoder_msg);
ros::Publisher pub_l("lwheel", &encoder_msg);

ros::Subscriber<std_msgs::Float32> sub_r("rwheel_vtarget", &cmd_vel_r_cb);
ros::Subscriber<std_msgs::Float32> sub_l("lwheel_vtarget", &cmd_vel_l_cb);

void count_r() 
{
  rCounter++;
}

void count_l() 
{
  lCounter++;
}

void setup()
{
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    double K = 1000.0 / (20 / (0.065 * 3.14159)); 
                  // (ms/s) / (ticksPerMeter)
                  // ticksPerMeter = ticksPerRev / MetersPerRev
    rPID.SetOutputLimits(0, 255);
    rPID.SetSampleTime(50);
    rPID.SetWheelParam(K);
    rPID.SetMode(AUTOMATIC);
    lPID.SetOutputLimits(0, 255);
    lPID.SetSampleTime(50);
    lPID.SetWheelParam(K);
    lPID.SetMode(AUTOMATIC);

    pinMode(rEncoder, INPUT);
    attachInterrupt(digitalPinToInterrupt(rEncoder), count_r, RISING);
    pinMode(lEncoder, INPUT);
    attachInterrupt(digitalPinToInterrupt(lEncoder), count_l, RISING);

    lastMessage = millis();

    pinMode(rMotor, OUTPUT);
    pinMode(lMotor, OUTPUT);

    nh.initNode();
    nh.advertise(pub_r);
    nh.advertise(pub_l);
    nh.subscribe(sub_r);
    nh.subscribe(sub_l);
    delay(10);
}

void loop()
{
    rPID.ComputeVelocity(rCounter);
    lPID.ComputeVelocity(lCounter);
    
    analogWrite(rMotor, rOutput);
    analogWrite(lMotor, lOutput);

    unsigned long now = millis();
    if((now - lastMessage) > serialPing) { // send odometry
        encoder_msg.data = rCounter;
        pub_r.publish(&encoder_msg);
        encoder_msg.data = lCounter;
        pub_l.publish(&encoder_msg);
        
        lastMessage = now;
    }

    nh.spinOnce();
}

    // roscore
    // rostopic pub rwheel_vtarget std_msgs/Float32  <speed>

    // noInterrupts();
    // // critical, time-sensitive code here
    // interrupts();

