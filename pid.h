#ifndef __PID_H__
#define __PID_H__

/* C251 does not support stdint.h; use native types instead */
typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;
typedef unsigned long  uint32_t;
typedef signed char    int8_t;
typedef signed int     int16_t;
typedef signed long    int32_t;

/**
 * @brief PID mode enumeration
 */
typedef enum {
    PID_MODE_POSITION = 0,  // Position PID (for servo/position control)
    PID_MODE_INCREMENTAL    // Incremental PID (for motor speed control)
} PidMode_t;

/**
 * @brief PID controller structure, simulating object-oriented class
 */
typedef struct {
    // PID parameters
    float kp;      // Proportional gain
    float ki;      // Integral gain
    float kd;      // Derivative gain
    
    // Limit values
    float maxOutput;   // Maximum output
    float minOutput;   // Minimum output
    
    // State variables
    float setpoint;     // Target value
    float lastError;   // Last error
    float lastLastError; // Last last error (for incremental PID)
    float integral;     // Integral value
    float lastOutput;   // Last output (for incremental PID)
    
    // PID mode
    PidMode_t mode;     // PID mode (position or incremental)
    
    // Method pointers (simulating member functions)
    void (*init)(void* self, float kp, float ki, float kd);
    float (*update)(void* self, float currentValue);
    void (*setSetpoint)(void* self, float setpoint);
    void (*setTunings)(void* self, float kp, float ki, float kd);
    void (*setLimits)(void* self, float maxOut, float minOut);
    void (*reset)(void* self);
    void (*setMode)(void* self, PidMode_t mode);
} Pid_t;

Pid_t* pidCreate(void);

void pidDestroy(Pid_t* pid);

/* Powered By LuoDiary*/

#endif /* PID_H */
