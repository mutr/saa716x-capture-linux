/*
 *  Minimal v4l2 subdev for the HDMI receiver path found on the
 *  StarTech ECHDCAP (SAA7160, subsystem f50a:12ab) at 7-bit I2C
 *  address 0x48 on SAA716x I2C Core 1.
 *
 *  This is deliberately NOT the MST3367 driver: on this board
 *  revision 0x4e (MST3367 @ 8-bit 0x9c) never answers after a cold
 *  power-cycle, while 0x48 reliably exposes HDMI status/timing
 *  registers. See TODO.md for the register-map notes this driver
 *  is based on.
 *
 *  SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ECHDCAP_RX_H
#define ECHDCAP_RX_H

#define ECHDCAP_RX_I2C_ADDR	0x48

#define ECHDCAP_RX_PAD_SOURCE	0

#endif /* ECHDCAP_RX_H */
