/** @file
 *	@brief MAVLink comm protocol built from ualberta.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 */
#ifndef MAVLINK_H
#define MAVLINK_H

#ifndef MAVLINK_STX
#define MAVLINK_STX 85
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_BIG_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 0
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 0
#endif

#include "version.h"
#include "ualberta.h"

#endif // MAVLINK_H
