/**
 * @file RTClient.h
 * @brief Main Client for Bionic Arm
 * @date 2018-11-28.
 * @author Junho Park
 * @version 1.0.0
 */

#ifndef RTCLIENT_H_
#define RTCLIENT_H_

#include <algorithm>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>		// string function definitions
#include <fcntl.h>		// File control definitions
#include <errno.h>		// Error number definitions
#include <termios.h>	// POSIX terminal control definitions
#include <time.h>		// time calls
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <math.h>

#include "NRMKsercan_tp.h"  //<Header for the SERCAN API
#include "NRMKhw_tp.h"		//<Header for the SERCAN API
//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/queue.h>
#include <rtdk.h>		//The rdtk real-time printing library
/****************************************************************************/

#include "EcatDataSocket.h"
#include "EcatControlSocket.h"

#include "EcatSystem/RTControlSocket.h"
#include "EcatSystem/Ecat_Master.h"
#include "EcatSystem/Ecat_Elmo.h"
#include "EcatSystem/Ecat_KistFinger.h"
#include "EcatSystem/Ecat_KistSensor.h"
#include "Control/Controller.h"
#include "Control/Motion.h"
#include "Control/Trajectory.h"
#include "KDL/SerialManipulator.h"
#include "Control/KistHand.h"
#include "devMouseObject.h"

#include "Poco/Net/ServerSocket.h"
#include "Poco/Net/SocketAddress.h"

#define WAKEUP_TIME				(5)				/**<Initial waiting time*/
#define NSEC_PER_SEC 			1000000000		/**<Expression of second in nano second*/

#define _DEBUG_ 			/**<Debug Print Parameter*/
#define _ECAT_ON_ 			/**<EtherCAT device enable Parameter*/
#define _USE_DC_MODE_		/**<EtherCAT Distributed Clock mode enable Parameter*/
#define _CAN_ON_
//#define _TCP_ON_
#define _RS232_ON_
//#define _PLOT_ON_
//#define _DEV_MOUSE_ON_

typedef unsigned int UINT32;	/**<typedef uint32_t*/
typedef int64_t		INT64;		/**<typedef uint64_t*/
typedef int32_t 	INT32;		/**<typedef int32_t*/
typedef int16_t 	INT16;		/**<typedef int16_t*/
typedef uint16_t 	UINT16;		/**<typedef uint16_t*/
typedef uint8_t 	UINT8;		/**<typedef uint8_t*/
typedef int8_t 		INT8;   	/**<typedef int8_t*/


// Cycle time in nanosecond
unsigned int cycle_ns = 1000000;  	/**< 1 ms, Initial Value */
static int period = 1000000;		/**< 1 ms, Initial Value */


#endif /* RTCLIENT_H_ */
