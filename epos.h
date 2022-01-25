#ifndef EPOS_H
#define EPOS_H

// EPOS2 OD indexes
#define CONTROLWORD                 0x6040      // controlword is used for commanding the movement; uint16
#define MODE_OF_OPERATION           0x6060      // used for setting operationg mode of epos2 (Homing mode, PPM); int8
#define TARGET_POSITION             0x607A      // target position for the movement; int32
#define PROFILE_VELOCITY            0x6081      // velocity of the movement; uint32
#define PROFILE_ACCELERATION        0x6083      // acceleration of the movement; uint32
#define PROFILE_DECELERATION        0x6084      // deceleration of the movement; uint32
#define MOTION_PROFILE_TYPE         0x6086      // motion type: 0 - linear, 1- sin^2; int16

// Bit masks. NOTE: it is not currently known whether it is lsb or msb format of storage. Currently written for msb, mirroring may be needed
#define STATUSWORD_FINISH_BIT       0x0400      // movement is finished
#define STATUSWORD_ERR_BIT          0x2000      // movement error
#define STATUSWORD_ACK_BIT          0x1000      // target position acknowledged
#define STATUSWORD_WARN_BIT         0x0080      // warning bit

#define CONTROLWORD_PPM_ENDLESS_BIT 0x8000      // for PPM sets endless movement
#define CONTROLWORD_HALT_BIT        0x0100      // halt bit
#define CONTROLWORD_PPM_ABS_BIT     0x0040      // for PPM defines if position is absolute or relative
#define CONTROLWORD_PPM_IMM_BIT     0x0020      // for PPM defines if target position should be changed immediately
#define CONTROLWORD_PPM_NEW_SETP_BIT    0x0010  // for PPM indicates that a new setpoint is valid 
#define CONTROLWORD_ENABLE_OP_BIT   0x0008      // enable operation

// constants
#define SHUTDOWN                    0x0006      // perform the shutdown of EPOS2
#define SWITCH_ON                   0x000F      // Switch on & enable
#define PPM_ABS                     0x001F      // in PPM set absolute position
#define PPM_ABS_IMM                 0x003F      // in PPM set absolute position and start immediately
#define PPM_REL                     0x005F      // in PPM set relative position
#define PPM_REL_IMM                 0x007F      // in PPM set relative position and start immediately

#endif //EPOS_H