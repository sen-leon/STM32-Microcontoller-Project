# STM32-Microcontoller-Project
Project for Electronics for Aerospace Engineers/Projekt für Elektronik für Raumfahrtingenieure

## Task Description:
*Create a Rotation Instrument*
- Continuous measurement of orientation (degrees from north) and rotation speed (degrees per second)
- When the blue USER button is pressed, the current orientation is set as the zero value ("tare")
- When the sensor rotates clockwise, the blue LED (LED4) should light up
- When the sensor rotates counterclockwise, the green LED (LED3) should light up
- The respective LED should become brighter when the sensor is turned faster
- If the sensor is within +/- 5 degrees of the zero value (see above), the respective LED should flash, otherwise it should light up continuously (for the human eye).
- The program logic should be "interrupt-controlled"

For more information, please refer to the documentation written in German [STM32_Kurzdokumentation_Leon_Senguen.pdf]
