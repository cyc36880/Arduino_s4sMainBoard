/**
 * Read the gyroscope's XY-axis angles
 */
#include "s4s_mainBoard.h"
#include "stdio.h"

s4s_mainBoard mainBoard;

void setup() 
{
    Serial.begin(115200);
    mainBoard.begin();

    // Enable gyroscope
    mainBoard.gyro_enable(true);
}

void loop() 
{
    char print_buffer[100] = {0};
    int16_t angleX, angleY;

    // Get gyroscope's XY-axis angles, not Z-axis
    mainBoard.gyro_get_angle(&angleX, &angleY, NULL);

    sprintf(print_buffer, "angleX: %d, angleY: %d\n", angleX, angleY);
    Serial.print(print_buffer);
    delay(100);
}
