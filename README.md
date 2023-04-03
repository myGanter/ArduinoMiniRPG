### Arduino MiniRPG

A retro-style game written for the Arduino platform.

Gameplay: you play as a hero whose goal is to go down into the dungeon as deep as possible, at each level you will be met by a portal that will take you to the level below. The lower, the more enemies. To counter them, the hero has several bombs that he can also find on the level.

The hero is controlled using a gyroscope, setting a bomb, using a button.

![GAME](https://github.com/myGanter/ArduinoMiniRPG/blob/master/images/Wokwi1.PNG)
![REALGAME](https://github.com/myGanter/ArduinoMiniRPG/blob/master/images/Real1.jpg)
![REALGAME](https://github.com/myGanter/ArduinoMiniRPG/blob/master/images/Real2.jpg)

### Components and modules
- arduino-nano
- lcd2004 or lcd1602
- resistors
- buzzer
- button
- mpu6050

### The connection diagram is available on the Wokwi page
- with lcd2004 module
    - https://wokwi.com/projects/350656075353031252
- with lcd1602 module
    - https://wokwi.com/projects/360981847741806593

In the Wokwi environment, there is an unpleasant bug with redrawing graphics, in real life I do not observe this problem.

## Settings in the code
    #define W 10    // Width of the generated maze
    #define H 10    // The height of the generated maze
    #define LcdW 20 // Horizontal matrix size (for lcd1602 set 16)
    #define LcdH 4  // Vertical matrix size (for lcd1602 set 2)
    // you probably won't need the rest of the settings :3