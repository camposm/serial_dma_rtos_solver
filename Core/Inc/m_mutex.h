/*
 * m_mutex.h
 *
 *  Created on: 14 de jul de 2020
 *      Author: mark
 */

#include <stdint.h>

#ifndef INC_M_MUTEX_H_
#define INC_M_MUTEX_H_

typedef uint32_t mutex_t __attribute__ ((aligned (4)));

extern void lock_mutex (mutex_t* mutex);
extern void unlock_mutex (mutex_t* mutex);

#endif /* INC_M_MUTEX_H_ */
