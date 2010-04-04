#ifndef BRAVO_BUTTONS_H
#define BRAVO_BUTTONS_H

#include <linux/input.h>

// Screen Dimensions
#define BRAVO_TOUCH_SCREEN_HEIGHT          (272)
#define BRAVO_TOUCH_SCREEN_WIDTH           (480)

#define BRAVO_BUTTON_ROW_TOP			   (BRAVO_TOUCH_SCREEN_HEIGHT - BRAVO_BUTTON_HEIGHT)
#define BRAVO_BUTTON_ROW_BOTTOM            (BRAVO_TOUCH_SCREEN_HEIGHT)

#define BRAVO_BUTTON_BOTTOM_OFFSET         (0)

// Max & Min Coordinate Values
#define BRAVO_TOUCH_SCREEN_X_MIN           (0)
#define BRAVO_TOUCH_SCREEN_X_MAX           (BRAVO_TOUCH_SCREEN_WIDTH  - 1)

#define BRAVO_TOUCH_SCREEN_Y_MIN           (0)
#define BRAVO_TOUCH_SCREEN_Y_MAX           (BRAVO_TOUCH_SCREEN_HEIGHT - 1)

// Button IDs
#define BRAVO_BUTTON_ID_HOME               KEY_HOME
#define BRAVO_BUTTON_ID_NONE			   0

// Button Sizes
// We have 5 buttons that span the entire width of the screen.
// The button height is currently arbitrary.
#define BRAVO_BUTTON_WIDTH                 (BRAVO_TOUCHSCREEN_WIDTH)
#define BRAVO_BUTTON_HEIGHT                (10)

#endif // BRAVO_BUTTONS_H
