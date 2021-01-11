/*
 * m_mutex.s
 *
 *  Created on: 14 de jul de 2020
 *      Author: mark
 */

// from https://developer.arm.com/documentation/dht0008/a/arm-synchronization-primitives/practical-uses/implementing-a-mutex
.equ locked,1
.equ unlocked,0

//
// lock_mutex
// extern void lock_mutex(mutex_t* mutex);
//
.global lock_mutex

lock_mutex:
    ldr         r1, =locked
_reload:
    ldrex       r2, [r0]
    cmp         r2, r1        // Test if mutex is locked or unlocked
    beq         _reload       // If locked - wait for it to be released, from 2
    strex       r2, r1, [r0]  // Not locked, attempt to lock it
    cmp         r2, #1        // Check if Store-Exclusive failed
    beq         _reload       // Failed - retry from 1
    // Lock acquired
    dmb                       // Required before accessing protected resource
    bx          lr

//
// unlock_mutex
// extern void unlock_mutex(mutex_t* mutex)
//
.global unlock_mutex

unlock_mutex:
    ldr     r1, =unlocked
    dmb                     // Required before releasing protected resource
    str     r1, [r0]        // unlock mutex
    bx      lr

