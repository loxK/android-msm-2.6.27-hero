/*
 * Definitions for Touch panel calibration algorithm.
 */
#ifndef __CALIBRATE_H
#define __CALIBRATE_H

#define TP_X_MIN 0
#define TP_X_MAX 1023
#define TP_Y_MIN 0
#define TP_Y_MAX 1023


// Calibration algorithm functions
void calibration_show(char *buf);
void calibration_store(const char *buf);
void calibration_points_show(char *buf);
void calibration_points_store(const char *buf);
void calibration_screen_show(char *buf);
void calibration_screen_store(const char *buf);
void calibration_init(void);
void calibration_check_mode(void);
void calibration_translate(int x, int y, int *rx, int *ry);

#endif /* __CALIBRATE_H */
