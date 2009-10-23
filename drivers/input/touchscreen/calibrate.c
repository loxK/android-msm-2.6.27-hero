/* drivers/input/touchscreen/calibrate.c
 *
 * Copyright (C) 2008 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <mach/msm_tssc.h>
#include <mach/board.h>

#define CALIBRATION_SCREEN_WIDTH 240
#define CALIBRATION_SCREEN_HEIGHT 320
#define CALIBRATION_POINTS 5
#define CALIBRATION_COORDINATES (CALIBRATION_POINTS * 2)
#define CALIBRATION_SAMPLE_SIZE 20
#define CALIBRATION_X_GAP 5
#define CALIBRATION_Y_GAP 5
// 5 points coordinates:
//  Point 1 - (15, 20)
//  Point 2 - (223, 20)
//  Point 3 - (119, 159)
//  Point 4 - (15, 298)
//  Point 5 - (223, 298)
#define TP_CALIBRATION_POINT1_X 15
#define TP_CALIBRATION_POINT1_Y 20
#define TP_CALIBRATION_POINT2_X 223
#define TP_CALIBRATION_POINT2_Y 20
#define TP_CALIBRATION_POINT3_X 15
#define TP_CALIBRATION_POINT3_Y 298
#define TP_CALIBRATION_POINT4_X 223
#define TP_CALIBRATION_POINT4_Y 298
#define TP_CALIBRATION_POINT5_X 119
#define TP_CALIBRATION_POINT5_Y 159
#define CALIBRATION_X_RANGE 780
#define CALIBRATION_Y_RANGE 820
#define CALIBRATION_ERROR 30//15//10

// Flag to decide wheather current TP need to do adjustment or not.
//  0: No, not necessary to do adjustment.
//  1: Yes, should use calibration algorithm to do adjustment.
static int adjustment_flag = 0;

static int calibration_flag = 0;
static int calibration_count = 0;
static int calibration_index = 0;

// calibration_direction: Raw data vs. screen coordinate direction.
//  0: No swaped. X, Y same direction.
//  1: No swaped. X, same direction, Y reversed.
//  2: No swaped. X, reversed, Y same direction.
//  3: No swaped. X reversed, Y reversed.
//  4: X, Y swaped. X, Y same direction.
//  5: X, Y swaped. X, same direction, Y reversed.
//  6: X, Y swaped. X, reversed, Y same direction.
//  7: X, Y swaped. X, same direction, Y reversed.
static int calibration_x_direction = 0;
static int calibration_y_direction = 0;
static int calibration_swapped = 0;
static int calibration_x_min = TP_X_MIN;
static int calibration_x_max = TP_X_MAX;
static int calibration_y_min = TP_Y_MIN;
static int calibration_y_max = TP_Y_MAX;
static int saved_x_min = TP_X_MIN;
static int saved_x_max = TP_X_MAX;
static int saved_y_min = TP_Y_MIN;
static int saved_y_max = TP_Y_MAX;

static int calibration_x[CALIBRATION_POINTS];
static int calibration_y[CALIBRATION_POINTS];
static int saved_x[CALIBRATION_POINTS];
static int saved_y[CALIBRATION_POINTS];
static int screen_x[CALIBRATION_POINTS];
static int screen_y[CALIBRATION_POINTS];

/*
 * Get the touch input raw data and store into the coordinates buffer.
 */
void calibration_get_points(int x, int y)
{
	int dx;
	int dy;

	if (calibration_index >= CALIBRATION_POINTS)
		return;

	dx = abs(calibration_x[calibration_index] - x);
	dy = abs(calibration_y[calibration_index] - y);
	
	if ((dx < CALIBRATION_X_GAP) && (dy < CALIBRATION_Y_GAP)) 
		calibration_count++;
	
	calibration_x[calibration_index] = x;
	calibration_y[calibration_index] = y;
}

/*
 * Translate the touch input raw data to the calibrated data.
 */
void calibration_translate(int x, int y, int *rx, int *ry)
{
	int cx, cy;
	
	if (calibration_flag > 0 && calibration_flag <= CALIBRATION_POINTS) { /* Under calibration */
		calibration_get_points(x, y);
		*rx = x;
		*ry = y;
	} else { /* Normal touch input */
		if (adjustment_flag) { /* After calibration */
			int kx = 0; 
			int ky = 0;
			
			if (calibration_swapped) { /* X, Y swapped */
				// Ratio translation.
				if ((calibration_x_max > calibration_x_min) && (calibration_y_max > calibration_y_min)) {
					kx = TP_X_MIN + ((x - calibration_y_min) * (TP_X_MAX - TP_X_MIN) / (calibration_y_max - calibration_y_min));
					ky = TP_Y_MIN + ((y - calibration_x_min) * (TP_Y_MAX - TP_Y_MIN) / (calibration_x_max - calibration_x_min));
				}            
				
				// X, Y direction
				cx = (calibration_x_direction) ? (ky) : (TP_X_MAX - ky);
				cy = (calibration_y_direction) ? (kx) : (TP_Y_MAX - kx);
			} else {
				// Ratio translation.
				if ((calibration_x_max > calibration_x_min) && (calibration_y_max > calibration_y_min)) {
					kx = TP_X_MIN + ((x - calibration_x_min) * (TP_X_MAX - TP_X_MIN) / (calibration_x_max - calibration_x_min));
					ky = TP_Y_MIN + ((y - calibration_y_min) * (TP_Y_MAX - TP_Y_MIN) / (calibration_y_max - calibration_y_min));
				}            
				
				// X, Y direction
				cx = (calibration_x_direction) ? (kx) : (TP_X_MAX - kx);
				cy = (calibration_y_direction) ? (ky) : (TP_Y_MAX - ky);
			}
			
			if (cx < TP_X_MIN)
				cx = TP_X_MIN;
			else if (cx > TP_X_MAX)
				cx = TP_X_MAX;

			if (cy < TP_Y_MIN)
				cy = TP_Y_MIN;
			else if (cy > TP_Y_MAX)
				cy = TP_Y_MAX;
			
			*rx = cx;
			*ry = cy;
		} else { /* No need to do calibration */
			*rx = y; /* Current device, x and y swapped */
			*ry = x;
		}
	}
}

/*
 * Show the calibration parameters.
 */
void calibration_show_parameters(int status)
{
	int i;
	
	for (i = 0; i < CALIBRATION_POINTS; i++)
		printk(KERN_INFO "touch_calibration: Point %d %d, %d.\n",
				i+1, calibration_x[i], calibration_y[i]);

	printk(KERN_INFO "touch_calibration: do calibration %s.\n",
			status ? "error" : "ok");
	printk(KERN_INFO "touch_calibration: x max %d, x min %d.\n",
			calibration_x_min, calibration_x_max);
	printk(KERN_INFO "touch_calibration: y max %d, y min %d.\n",
			calibration_y_min, calibration_y_max);
        printk(KERN_INFO "touch_calibration: %s x %s, y %s.\n", 
        	calibration_swapped ? "X Y swapped," : "", 
        	calibration_x_direction ? "reversed" : "same direction", 
        	calibration_y_direction ? "reversed" : "same direction");
}
	
/*
 * Calculate calibration variables.
 */
void calibration_calculate_varialbes(void)
{
	int xdir, xlen, ydir, ylen;
	int x_length, y_length;
	int x1, x2, y1, y2;
	int i;
	int result = 0;
	
	saved_x_min = calibration_x_min;
	saved_x_max = calibration_x_max;
	saved_y_min = calibration_y_min;
	saved_y_max = calibration_y_max;
	
	if (calibration_index < CALIBRATION_POINTS - 1) {
		result = 1; /* Calibration points not enough */
	} else {
		if (calibration_x[0] > calibration_x[1]) {
			xdir = 0; /* Reversed */
			xlen = calibration_x[0] - calibration_x[1];
		} else {
			xdir = 1; /* Same */
			xlen = calibration_x[1] - calibration_x[0];
		}
    
		if (calibration_y[0] > calibration_y[1]) {
			ydir = 0; /* Reversed */
			ylen = calibration_y[0] - calibration_y[1];
		} else {
			ydir = 1; /* Same */
			ylen = calibration_y[1] - calibration_y[0];
		}
        
		if (xlen > ylen) {
			calibration_swapped = 0; /* No swapped */
			calibration_x_direction = xdir;
			x_length = xlen;
			
			if (calibration_y[0] > calibration_y[3]) {
				calibration_y_direction = 0; /* Reversed */
				y_length = calibration_y[0] - calibration_y[3];
			} else {
				calibration_y_direction = 1; /* Same */
				y_length = calibration_y[3] - calibration_y[0];
			}
			
			x1 = (calibration_x_direction) ? (calibration_x[0]) : (calibration_x[1]);
			x2 = (calibration_x_direction) ? (calibration_x[1]) : (calibration_x[0]);
			y1 = (calibration_y_direction) ? (calibration_y[0]) : (calibration_y[3]);
			y2 = (calibration_y_direction) ? (calibration_y[3]) : (calibration_y[0]);
		} else {
			calibration_swapped = 1; /* X, Y swapped */
			calibration_x_direction = ydir;
			x_length = ylen;
			
			if (calibration_x[0] > calibration_x[3]) {
				calibration_y_direction = 0; /* Reversed */
				y_length = calibration_x[0] - calibration_x[3];
			} else {
				calibration_y_direction = 1; /* Same */
				y_length = calibration_x[3] - calibration_x[0];
			}
			
			x1 = (calibration_x_direction) ? (calibration_y[0]) : (calibration_y[1]);
			x2 = (calibration_x_direction) ? (calibration_y[1]) : (calibration_y[0]);
			y1 = (calibration_y_direction) ? (calibration_x[0]) : (calibration_x[3]);
			y2 = (calibration_y_direction) ? (calibration_x[3]) : (calibration_x[0]);
		}
    
		xlen = (screen_x[1] - screen_x[0]);
		
		if (xlen) {
			calibration_x_min = x1 - ((x_length) * (screen_x[0]) / xlen);
			calibration_x_max = x2 + ((x_length) * (CALIBRATION_SCREEN_WIDTH-screen_x[1]) / xlen);
		} else {
			result = 1;
		}
        
		ylen = (screen_y[2] - screen_y[0]);
		
		if (ylen) {
			calibration_y_min = y1 - ((y_length) * (screen_y[0]) / ylen);
			calibration_y_max = y2 + ((y_length) * (CALIBRATION_SCREEN_HEIGHT-screen_y[2]) / ylen);
		} else {
			result = 1;
		}
	}
    	printk("mfg_mode=%d\n", board_mfg_mode());
    	if(board_mfg_mode()==1)
    	{
    	    printk("mfg_mode=factory2\n");
    	    
    	    #if 0
    	    printk("calibration_x_max-calibration_x_min=%d calibration_y_max-calibration_y_min=%d\n"
    	    , calibration_x_max-calibration_x_min, calibration_y_max-calibration_y_min); 
    	    printk("abs(CALIBRATION_X_RANGE-(calibration_x_max-calibration_x_min))=%d\n", abs(CALIBRATION_X_RANGE-(calibration_x_max-calibration_x_min)));       
    	    printk("abs(CALIBRATION_Y_RANGE-(calibration_y_max-calibration_y_min))=%d\n", abs(CALIBRATION_Y_RANGE-(calibration_y_max-calibration_y_min)));       
    	    if(abs(CALIBRATION_X_RANGE-(calibration_x_max-calibration_x_min))>CALIBRATION_ERROR
    	    || abs(CALIBRATION_Y_RANGE-(calibration_y_max-calibration_y_min))>CALIBRATION_ERROR
    	    ) result=1;
    	    #endif
    	}
        
	if (calibration_x_min > 250 || calibration_x_max < 700 || calibration_y_min > 250 || calibration_y_max < 700)
		result = 1; /* Not in the acceptable area */
	
	calibration_show_parameters(result);

	if (result == 1) {
		for (i=0; i<CALIBRATION_POINTS; i++) {
			calibration_x[i] = saved_x[i];
			calibration_y[i] = saved_y[i];
		}
	
		calibration_x_min = saved_x_min;
		calibration_x_max = saved_x_max;
		calibration_y_min = saved_y_min;
		calibration_y_max = saved_y_max;
		
		calibration_flag = 9; /* Calibration error */
	} else {
		for (i=0; i<CALIBRATION_POINTS; i++) {
			saved_x[i] = calibration_x[i];
			saved_y[i] = calibration_y[i];
		}
		
		calibration_flag = 8; /* Calibration ok */
	}        

	adjustment_flag = 1; /* Calibration finished */
}

/*
 * Set the coordinates of calibration points from input data buffer.
 */
void calibration_set(int *inPtr)
{
	int i;
	int j = 0;
	
	for (i = 0; i < CALIBRATION_POINTS; i++) {
		calibration_x[i] = inPtr[j++];
		calibration_y[i] = inPtr[j++];
	}
}

/*
 * Initialize calibration parameters at boot time.
 */
void calibration_init(void)
{
	screen_x[0] = TP_CALIBRATION_POINT1_X;
	screen_x[1] = TP_CALIBRATION_POINT2_X;
	screen_x[2] = TP_CALIBRATION_POINT3_X;
	screen_x[3] = TP_CALIBRATION_POINT4_X;
	screen_x[4] = TP_CALIBRATION_POINT5_X;
	screen_y[0] = TP_CALIBRATION_POINT1_Y;
	screen_y[1] = TP_CALIBRATION_POINT2_Y;
	screen_y[2] = TP_CALIBRATION_POINT3_Y;
	screen_y[3] = TP_CALIBRATION_POINT4_Y;
	screen_y[4] = TP_CALIBRATION_POINT5_Y;
}

/*
 * Display the flag of the calibration algorithm.
 * 	0: exit calibration mode.
 * 	1: enter calibration mode.
 *	2: calibration error.
 *	3: calibration ok.
 */
ssize_t calibration_show(char *buf)
{
	return sprintf(buf, "%d\n", calibration_flag);
}

/*
 * Store the flag of the calibration algorithm.
 */
void calibration_store(const char *buf)
{
	char *ptr_data = (char *)buf;
	unsigned long val = simple_strtoul(ptr_data, NULL, 10);

	if (val == 0) { /* exit calibration mode */
		calibration_flag = 0;
		calibration_count = 0;
		calibration_calculate_varialbes();
		calibration_index = 0;
		printk(KERN_DEBUG "touch_calibration: exit.\n");
	} else { /* calibration mode */
		calibration_flag = val;
		calibration_index = val - 1;
		calibration_count = 0;
		
		if (val <= CALIBRATION_POINTS)
			printk(KERN_DEBUG "touch_calibration: point %d.\n", 
					calibration_index + 1);
		else
			printk(KERN_ERR "touch_calibration: point error.\n");
	}
}

/*
 * Display the coordinates of the calibration touch panel input points.
 */
ssize_t calibration_points_show(char *buf)
{
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			saved_x[0], saved_y[0], saved_x[1], saved_y[1], 
			saved_x[2], saved_y[2], saved_x[3], saved_y[3], 
			saved_x[4], saved_y[4]);
}

/*
 * Store the coordinates of the calibration touch panel input points.
 */
void calibration_points_store(const char *buf)
{
	char *ptr_data = (char *)buf;
	char *p;
	int cnt = 0;
	int tmp[CALIBRATION_COORDINATES];
	unsigned long val;
	
	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)			
			break;
		
		if (cnt >= CALIBRATION_COORDINATES)	
			break;
		
		val = simple_strtoul(p, NULL, 10);
		tmp[cnt++] = val;
	}

	if (cnt >= CALIBRATION_COORDINATES) {
		calibration_set(&tmp[0]);
		calibration_index = CALIBRATION_POINTS;
		calibration_calculate_varialbes();
		calibration_index = 0;
	}
}

/*
 * Display the coordinates of the calibration screen points.
 */
ssize_t calibration_screen_show(char *buf)
{
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			screen_x[0], screen_y[0], 
			screen_x[1], screen_y[1], screen_x[2], screen_y[2], 
			screen_x[3], screen_y[3], screen_x[4], screen_y[4]);
}

/*
 * Store the coordinates of the calibration screen points.
 */
void calibration_screen_store(const char *buf)
{
	char *ptr_data = (char *)buf;
	char *p;
	int cnt = 0;
	int tmp[CALIBRATION_COORDINATES];
	unsigned long val;

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)			
		    break;
		    
		if (cnt >= CALIBRATION_COORDINATES)	
		    break;

		val = simple_strtoul(p, NULL, 10);
		tmp[cnt++] = val;
	}

	if (cnt >= CALIBRATION_COORDINATES) {
		int i;
		int j = 0;
		
		for (i = 0; i < CALIBRATION_POINTS; i++) {
			screen_x[i] = tmp[j++];
			screen_y[i] = tmp[j++];
		}
	}
}
