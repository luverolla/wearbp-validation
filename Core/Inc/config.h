/*
 * config.h
 *
 *  Created on: May 20, 2024
 *      Author: Luigi Verolla
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define CFG_WINSIZE 30 // s
#define CFG_FSAMPLE 125 // Hz (samples/s)

#define CFG_CHUNKLEN (1*CFG_FSAMPLE) // samples
#define CFG_SIGLEN   (CFG_WINSIZE*CFG_FSAMPLE) // samples

#endif /* INC_CONFIG_H_ */
