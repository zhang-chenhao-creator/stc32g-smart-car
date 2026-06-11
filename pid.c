#include "pid.h"
#include <stdlib.h>

/**
 * @brief Initialize PID controller
 * @param self Pointer to PID controller instance
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param maxOut Maximum output value
 * @param minOut Minimum output value
 */
static void pidInit(void* self, float kp, float ki, float kd) {
    Pid_t* pid = (Pid_t*)self;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->maxOutput = 100.0f;
    pid->minOutput = -100.0f;
    pid->setpoint = 0.0f;
    pid->lastError = 0.0f;
    pid->lastLastError = 0.0f;
    pid->integral = 0.0f;
    pid->lastOutput = 0.0f;
    pid->mode = PID_MODE_POSITION;
}

/**
 * @brief Update position PID controller and calculate output
 * @param self Pointer to PID controller instance
 * @param currentValue Current process value
 * @return Calculated PID output
 */
static float pidUpdatePosition(void* self, float currentValue) {
    Pid_t* pid = (Pid_t*)self;
    
    // Calculate error
    float error = pid->setpoint - currentValue;
    
    // Calculate derivative
    float derivative = (error - pid->lastError) * pid->kd;
    
    // Calculate output before integral
    float output = pid->kp * error + derivative;
    
    // Calculate integral with anti-windup
    float tempIntegral = pid->integral + error * pid->ki;
    float tempOutput = output + tempIntegral;
    
    // Anti-windup: only update integral if output is within limits
    if (tempOutput <= pid->maxOutput && tempOutput >= pid->minOutput) {
        pid->integral = tempIntegral;
        output = tempOutput;
    } else {
        // Output is saturated, don't update integral
        output = pid->kp * error + pid->integral + derivative;
        
        // Limit output range
        if (output > pid->maxOutput) {
            output = pid->maxOutput;
        } else if (output < pid->minOutput) {
            output = pid->minOutput;
        }
    }
    
    // Save current error
    pid->lastError = error;
    pid->lastOutput = output;
    
    return output;
}

/**
 * @brief Update incremental PID controller and calculate output
 * @param self Pointer to PID controller instance
 * @param currentValue Current process value
 * @return Calculated PID output (incremental)
 */
static float pidUpdateIncremental(void* self, float currentValue) {
    Pid_t* pid = (Pid_t*)self;
    
    // Calculate current error
    float error = pid->setpoint - currentValue;
    
    // Calculate incremental output
    // Δu(k) = Kp * (e(k) - e(k-1)) + Ki * e(k) + Kd * (e(k) - 2*e(k-1) + e(k-2))
    float deltaP = pid->kp * (error - pid->lastError);
    float deltaI = pid->ki * error;
    float deltaD = pid->kd * (error - 2.0f * pid->lastError + pid->lastLastError);
    
    float deltaOutput = deltaP + deltaI + deltaD;
    float output = pid->lastOutput + deltaOutput;
    
    // Limit output range
    if (output > pid->maxOutput) {
        output = pid->maxOutput;
    } else if (output < pid->minOutput) {
        output = pid->minOutput;
    }
    
    // Update state variables
    pid->lastLastError = pid->lastError;
    pid->lastError = error;
    pid->lastOutput = output;
    
    return output;
}

/**
 * @brief Update PID controller (auto select mode) and calculate output
 * @param self Pointer to PID controller instance
 * @param currentValue Current process value
 * @return Calculated PID output
 */
static float pidUpdate(void* self, float currentValue) {
    Pid_t* pid = (Pid_t*)self;
    
    if (pid->mode == PID_MODE_INCREMENTAL) {
        return pidUpdateIncremental(self, currentValue);
    } else {
        return pidUpdatePosition(self, currentValue);
    }
}

/**
 * @brief Set PID controller setpoint
 * @param self Pointer to PID controller instance
 * @param setpoint Target value
 */
static void pidSetSetpoint(void* self, float setpoint) {
    Pid_t* pid = (Pid_t*)self;
    pid->setpoint = setpoint;
}

/**
 * @brief Set PID controller tunings
 * @param self Pointer to PID controller instance
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
static void pidSetTunings(void* self, float kp, float ki, float kd) {
    Pid_t* pid = (Pid_t*)self;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief Set PID controller output limits
 * @param self Pointer to PID controller instance
 * @param maxOut Maximum output value
 * @param minOut Minimum output value
 */
static void pidSetLimits(void* self, float maxOut, float minOut) {
    Pid_t* pid = (Pid_t*)self;
    pid->maxOutput = maxOut;
    pid->minOutput = minOut;
}

/**
 * @brief Reset PID controller state
 * @param self Pointer to PID controller instance
 */
static void pidReset(void* self) {
    Pid_t* pid = (Pid_t*)self;
    pid->lastError = 0.0f;
    pid->lastLastError = 0.0f;
    pid->integral = 0.0f;
    pid->lastOutput = 0.0f;
}

/**
 * @brief Set PID controller mode
 * @param self Pointer to PID controller instance
 * @param mode PID mode (position or incremental)
 */
static void pidSetMode(void* self, PidMode_t mode) {
    Pid_t* pid = (Pid_t*)self;
    pid->mode = mode;
    pidReset(self);
}

/**
 * @brief Create a new PID controller instance
 * @return Pointer to the created PID controller
 */
Pid_t* pidCreate(void) {
    Pid_t* pid = (Pid_t*)malloc(sizeof(Pid_t));
    if (pid) {
        // Initialize method pointers
        pid->init = pidInit;
        pid->update = pidUpdate;
        pid->setSetpoint = pidSetSetpoint;
        pid->setTunings = pidSetTunings;
        pid->setLimits = pidSetLimits;
        pid->reset = pidReset;
        pid->setMode = pidSetMode;
        
        // Initialize parameters
        pidInit(pid, 0.0f, 0.0f, 0.0f);
    }
    return pid;
}

/**
 * @brief Destroy a PID controller instance
 * @param pid Pointer to the PID controller to destroy
 */
void pidDestroy(Pid_t* pid) {
    if (pid) {
        free(pid);
    }
}

/* Powered By LuoDiary*/
