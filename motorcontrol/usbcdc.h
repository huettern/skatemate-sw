/*
   SKATEMATE - Copyright (C) 2017 FHNW Project 4 Team 2
 */

/**
 * @file       usbcdc.h
 * @brief      USB CDC methods providing shell access and debugging
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       23 March 2017
 * 
 *
 * @addtogroup USBCDC
 * @{
 */
typedef struct
{
	const char* name;
	float* loc;
} usbcdcParameterStruct_t;

void usbcdcInit(void);
void usbcdcHandleShell(void);
void usbcdcSetShellVars(const usbcdcParameterStruct_t** vars);


/** @} */