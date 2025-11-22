/**
 * Encoder Motor Position Loop Control
 */
#include "s4s_mainBoard.h"
#include "stdio.h"

s4s_mainBoard mainBoard;

void setup()
{
    mainBoard.begin();

    // Set id 0 encoder motor mode
    mainBoard.encoder_motor_set_mode(0, 1);
}

void loop()
{
    // Set id 0 encoder motor position
    mainBoard.encoder_motor_set_position(0, 200);
    delay(2000);

    // Set id 0 encoder motor position
    mainBoard.encoder_motor_set_position(0, 0);
    delay(2000);
}
