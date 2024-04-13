#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Bool.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);
//   avoid using pins with LEDs attached

ros::NodeHandle nh;
ros::Publisher nh.advertise<std_msgs::EncoderMsg>("encoder_msgs", 10);

void setup()
{
    Serial.begin(9600);
    Serial.println("Basic Encoder Test:");
}

long oldPosition = -999;

void loop()
{
    long newPosition = myEnc.read();
    if (newPosition != oldPosition)
    {
        oldPosition = newPosition;
        Serial.println(newPosition);
    }
}
