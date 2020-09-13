
#ifndef __XENO__
#define __XENO__
#endif

#include "RTClient.h"

//Modify this number to indicate the actual number of motor on the network
#define ELMO_TOTAL 6
#define BIONIC_ARM_DOF 6
/****************************************************************************/
hyuEcat::Master ecatmaster;
hyuEcat::EcatElmo ecat_elmo[ELMO_TOTAL];
HYUControl::Trajectory traj5th;
/****************************************************************************/
int serial_fd = -1;
int sercan_fd = -1;

/****************************************************************************/
struct LOGGING_PACK
{
	double 	Time;						/**< Global Time						*/
	double 	ActualPos[BIONIC_ARM_DOF]; 	/**< Actual Position in Radian			*/
	double 	ActualVel[BIONIC_ARM_DOF];	/**< Actual Velocity in Radian/second	*/
	short  	ActualToq[BIONIC_ARM_DOF];
	double 	DesiredPos[BIONIC_ARM_DOF];
	double  DesiredVel[BIONIC_ARM_DOF];
	short  	DesiredToq[BIONIC_ARM_DOF];
};

// NRMKDataSocket for plotting axes data in Data Scope
EcatDataSocket datasocket;
devMouseObject dMouse;
/****************************************************************************/
// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double double_gt=0; //real global time
float float_dt=0;
/****************************************************************************/
// EtherCAT Data (Dual-Arm)
UINT16	StatusWord[BIONIC_ARM_DOF] = {0,};
INT32 	ActualPos[BIONIC_ARM_DOF] = {0,};
INT32 	ActualVel[BIONIC_ARM_DOF] = {0,};
INT16 	ActualTor[BIONIC_ARM_DOF] = {0,};
INT8	ModeOfOperationDisplay[BIONIC_ARM_DOF] = {0,};
INT8	DeviceState[BIONIC_ARM_DOF] = {0,};
INT16 	TargetTor[BIONIC_ARM_DOF] = {0,};		//100.0 persentage
INT32 	TargetPos[BIONIC_ARM_DOF] = {0, };
INT32 	TargetVel[BIONIC_ARM_DOF] = {0, };
/****************************************************************************/
// Xenomai RT tasks
RT_TASK RTArm_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK tcpip_task;
RT_TASK serial_task;
RT_TASK can_task;
RT_TASK can_rev_task;
RT_TASK devmouse_task;

RT_QUEUE msg_can;
RT_QUEUE msg_plot;
RT_QUEUE msg_tcpip;

static void signal_handler(int sig);

/****************************************************************************/
// For RT thread management
unsigned long fault_count=0;
unsigned long ethercat_time=0;
unsigned long worst_time=0;
/****************************************************************************/
static double ActualPos_Rad[BIONIC_ARM_DOF] = {0.0,};
static double ActualVel_Rad[BIONIC_ARM_DOF] = {0.0,};
static double TargetPos_Rad[BIONIC_ARM_DOF] = {0.0,};
static double TargetVel_Rad[BIONIC_ARM_DOF] = {0.0,};
static double TargetAcc_Rad[BIONIC_ARM_DOF] = {0.0,};
static double TargetToq[BIONIC_ARM_DOF] = {0.0,};
int TrajFlag[BIONIC_ARM_DOF] = {0,};

static int isSlaveInit(void)
{
	int elmo_count = 0;
	int slave_count = 0;

	for(int i=0; i<ELMO_TOTAL; ++i)
	{
		if(ecat_elmo[i].initialized())
		{
			elmo_count++;
		}
	}

	for(int j=0; j<((int)ecatmaster.GetConnectedSlaves()); j++)
	{
		if(ecatmaster.GetSlaveState(j) == 0x08)
		{
			slave_count++;
		}
	}

	if((elmo_count == ELMO_TOTAL) && (slave_count == ((int)ecatmaster.GetConnectedSlaves())))
		return 1;
	else
		return 0;
}

int j_homing = ELMO_TOTAL;
int HomingFlag = 0;
static int isElmoHoming(void)
{
	int num_traj=ELMO_TOTAL;
	VectorXd TargetInitPos(num_traj);
	VectorXd TargetInitVel(num_traj);
	VectorXd TargetFinalPos(num_traj);
	VectorXd CurrentPos(num_traj);
	VectorXd CurrentVel(num_traj);
	VectorXd CurrentAcc(num_traj);
	double TargetDuration = 5.0;

	CurrentPos.setZero();
	CurrentVel.setZero();
	//TargetInitPos << hominginfo[0].HomingOffset, hominginfo[1].HomingOffset,
	//		-hominginfo[2].HomingOffset, hominginfo[3].HomingOffset, hominginfo[4].HomingOffset;

	for(int i=0; i<ELMO_TOTAL; i++)
	{
		TargetInitPos(i) = ActualPos[i];
	}

	TargetInitVel.setZero();
	TargetFinalPos.setZero();
	TargetFinalPos(3) = (20.0/360.0)*4096.0*52.0;

	if(j_homing < 1)
	{
		HomingFlag = 1;
		for(int i=0;i<ELMO_TOTAL;i++){
			TrajFlag[i] = -1;
			if(i >= 4)
			{
				ecat_elmo[i].mode_of_operation_ = ecat_elmo[i].MODE_CYCLIC_SYNC_VELEOCITY;
			}
			else
			{
				ecat_elmo[i].mode_of_operation_ = ecat_elmo[i].MODE_CYCLIC_SYNC_TORQUE;
			}

		}
	}
	else
	{
		if(!ecat_elmo[j_homing-1].isHoming())  //check homing and change the mode to homing mode
		{
			ecat_elmo[j_homing-1].mode_of_operation_ = ecat_elmo[j_homing-1].MODE_HOMING;
			TrajFlag[j_homing-1] = 1;

		}
		else if(ecat_elmo[j_homing-1].isHoming() && TrajFlag[j_homing-1] == 1)
		{
			ecat_elmo[j_homing-1].mode_of_operation_ = ecat_elmo[j_homing-1].MODE_CYCLIC_SYNC_POSITION;
			ecat_elmo[j_homing-1].target_position_ = ActualPos[j_homing-1];
			if(ecat_elmo[j_homing-1].mode_of_operation_display_ == ecat_elmo[j_homing-1].MODE_CYCLIC_SYNC_POSITION)
			{
				rt_printf("\nTargetInitPos:%0.1f, TargetInitVel:%0.1f, TargetFinalPos:%0.1f\n",
						TargetInitPos(j_homing-1), TargetInitVel(j_homing-1), TargetFinalPos(j_homing-1));
				ecat_elmo[j_homing-1].target_position_ = ActualPos[j_homing-1];    //Keep the end of motion
				traj5th.SetPoly5th(double_gt, TargetInitPos, TargetInitVel, TargetFinalPos, TargetDuration, num_traj); //make trajectory to go to the 90 deg position
				TrajFlag[j_homing-1] = 2;
			}
		}
		else if(ecat_elmo[j_homing-1].isHoming() && TrajFlag[j_homing-1] == 2)
		{
			traj5th.Poly5th(double_gt, CurrentPos, CurrentVel, CurrentAcc);
			TargetPos[j_homing-1] = (int32_t)(round(CurrentPos(j_homing-1)));
			ecat_elmo[j_homing-1].writePosition(TargetPos[j_homing-1]);
			if( ActualPos[j_homing-1] == (int32_t)(TargetFinalPos(j_homing-1)) )
				TrajFlag[j_homing-1] = 3;
		}
		else if(ecat_elmo[j_homing-1].isHoming() && TrajFlag[j_homing-1] == 3)
		{
			j_homing--;
		}
	}

	return 0;
}

Vector3d ForwardPos[2];
Vector3d ForwardOri[2];
Vector3d ForwardAxis[2];
double TaskCondNumber[2];
double OrientCondNumber[2];
int NumChain;

// RTArm_task
void RTRArm_run(void *arg)
{

#if defined(_PLOT_ON_)
	int sampling_time 	= 20;	// Data is sampled every 10 cycles.
	int sampling_tick 	= sampling_time;

	void *msg;
	LOGGING_PACK logBuff;
	int len = sizeof(LOGGING_PACK);
#endif
	RTIME now, previous;
	RTIME p1 = 0;
	RTIME p3 = 0;

	short MaxTor = 1200;

	int reset_counter=0;
	int k=0;

	uint16_t ControlMotion = SYSTEM_BEGIN;
	uint16_t JointState = SYSTEM_BEGIN;

	VectorXd finPos(BIONIC_ARM_DOF);
	finPos.setZero();
	finPos(3) = 20.0;

	SerialManipulator BionicArm;
	HYUControl::Controller Control(&BionicArm);
	HYUControl::Motion motion(&BionicArm);

	BionicArm.UpdateManipulatorParam();

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	while (1)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle

		previous = rt_timer_read();

		ecatmaster.RxUpdate();

		for(k=0; k < ELMO_TOTAL; k++)
		{
			DeviceState[k] = 				ecat_elmo[k].Elmo_DeviceState();
			StatusWord[k] = 				ecat_elmo[k].status_word_;
			ModeOfOperationDisplay[k] = 	ecat_elmo[k].mode_of_operation_display_;
			ActualPos[k] = 					ecat_elmo[k].position_;
			ActualVel[k] = 					ecat_elmo[k].velocity_;
			ActualTor[k] = 					ecat_elmo[k].torque_;
		}

		if( system_ready )
		{

			BionicArm.ENCtoRAD(ActualPos, ActualPos_Rad);
			BionicArm.VelocityConvert(ActualVel, ActualVel_Rad);

			if(HomingFlag == 0)
			{
				isElmoHoming();
				//HomingFlag = 1;
			}
			else
			{
				BionicArm.pKin->PrepareJacobian(ActualPos_Rad);
				BionicArm.pDyn->PrepareDynamics(ActualPos_Rad, ActualVel_Rad);

				BionicArm.pKin->GetManipulability( TaskCondNumber, OrientCondNumber );
				BionicArm.pKin->GetForwardKinematics( ForwardPos, ForwardOri, NumChain );

				//BionicArm.StateMachine( ActualPos_Rad, ActualVel_Rad, finPos, JointState, ControlMotion );
				motion.JointMotion( TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, finPos, ActualPos_Rad, ActualVel_Rad, double_gt, JointState, ControlMotion );

				//Control.PDController( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetToq, float_dt );
				Control.PDGravController( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetToq);

				BionicArm.VelocityInvConvert(TargetVel, TargetVel_Rad);
				BionicArm.TorqueConvert(TargetToq, TargetTor, MaxTor);

				//write the motor data
				for(int j=0; j < ELMO_TOTAL; ++j)
				{

					if(double_gt >= 1.0)
					{
						if(j >= 4)
						{
							ecat_elmo[j].writeVelocity(0);
						}
						else
						{
							ecat_elmo[j].writeTorque(TargetTor[j]);
						}

					}
					else
					{
						if(j >= 4)
						{
							ecat_elmo[j].writeVelocity(0);
						}
						else
						{
							ecat_elmo[j].writeTorque(0);
						}
					}
				}
			}
		}

		ecatmaster.TxUpdate();

#if defined(_USE_DC_MODE_)
		ecatmaster.SyncEcatMaster(rt_timer_read());
#endif

		// For EtherCAT performance statistics
		p1 = p3;
		p3 = rt_timer_read();
		now = rt_timer_read();

		if ( isSlaveInit() == 1 )
		{
			reset_counter=1;
			float_dt = ((float)(long)(p3 - p1))*1e-3; 		// us
			double_gt += ((double)(long)(p3 - p1))*1e-9; 	// s
			ethercat_time = (long) now - previous;

			if( double_gt >= 0.5 )
			{
				system_ready=1;	//all drives have been done

				if ( worst_time<ethercat_time )
					worst_time=ethercat_time;

				if( ethercat_time > (unsigned long)cycle_ns )
				{
					fault_count++;
					worst_time=0;
				}
			}

#if defined(_PLOT_ON_)
			if ( (system_ready==1) && datasocket.hasConnection() && (sampling_tick-- == 0) )
			{
				sampling_tick = sampling_time - 1; // 'minus one' is necessary for intended operation

				logBuff.Time = double_gt;
				for(int k=0; k<BIONIC_ARM_DOF; k++)
				{
					logBuff.ActualPos[k] = ActualPos_Rad[k]*RADtoDEG;
					logBuff.ActualVel[k] = ActualVel_Rad[k]*RADtoDEG;
					logBuff.ActualToq[k] = ActualTor[k];

					logBuff.DesiredPos[k] = TargetPos_Rad[k]*RADtoDEG;
					logBuff.DesiredVel[k] = TargetVel_Rad[k]*RADtoDEG;
					logBuff.DesiredToq[k] = TargetTor[k];
				}

				msg = rt_queue_alloc(&msg_plot, len);
				if(msg == NULL)
					rt_printf("rt_queue_alloc Failed to allocate, NULL pointer received\n");

				memcpy(msg, &logBuff, len);
				rt_queue_send(&msg_plot, msg, len, Q_NORMAL);
			}
#endif
		}
		else
		{
			if(reset_counter == 0)
			{
				double_gt = 0;
			}

			if(ecatmaster.GetConnectedSlaves() < ELMO_TOTAL)
			{
				exit(EXIT_SUCCESS);
			}

			system_ready = 0;
			worst_time = 0;
			ethercat_time = 0;
		}
	}
}

float float_dt_serial=0;
unsigned long serial_time=0;
unsigned long serial_worst_time=0;
unsigned long serial_fault_count=0;

unsigned char kchr;
unsigned char chr;

void serial_task_proc(void *arg)
{
	int nbytes;

	RTIME now, previous;
	RTIME p1 = 0;
	RTIME p3 = 0;

	kchr =  'a';

	unsigned int serial_cycle_ns = 2e6;
	rt_task_set_periodic(NULL, TM_NOW, serial_cycle_ns);
	for(;;)
	{
		rt_task_wait_period(NULL);
		previous = rt_timer_read();

		if(system_ready)
		{
			//write
			if(NRMKkbhit())
			{
				write(serial_fd, &kchr, 1);
			}

			//read
			nbytes = read(serial_fd, &chr, 1);
			if(nbytes > 0)
			{
				if(kchr == 'a' && chr == 'f')
					kchr = 'b';
				else if(kchr == 'b' && chr == 'f')
					kchr = 'a';
			}

			now = rt_timer_read();
			float_dt_serial = ((float)(long)(p3 - p1))*1e-3; 		// us
			serial_time = (long) now - previous;

			if ( serial_worst_time<serial_time )
				serial_worst_time=serial_time;

			if( serial_time > (unsigned long)serial_cycle_ns )
			{
				serial_fault_count++;
				serial_worst_time=0;
			}
		}
		else
		{
			serial_worst_time = 0;
			serial_time = 0;
		}

		p1 = p3;
		p3 = rt_timer_read();
	}
}

float float_dt_dmouse=0;
unsigned long dmouse_time=0;
unsigned long dmouse_worst_time=0;
unsigned long dmouse_fault_count=0;

void devmouse_task_proc(void *arg)
{

	dMouse.Activate();

	int fd  = dMouse.getMousefd();;
	struct input_event ie = dMouse.getInputEvent();;
	struct mouse_t mouse = dMouse.getMouse();;
	struct fb_t fb = dMouse.getMousefb();

	ssize_t nread;

	RTIME now, previous;
	RTIME p1 = 0;
	RTIME p3 = 0;

	unsigned int devmouse_cycle_ns = 100e6;
	rt_task_set_periodic(NULL, TM_NOW, devmouse_cycle_ns);
	for(;;)
	{
		rt_task_wait_period(NULL);
		previous = rt_timer_read();

		if(system_ready)
		{

			if ( (nread = read(fd, &ie, sizeof(struct input_event))) > 0)
			{
				dMouse.print_event();
				dMouse.print_mouse_state();
				dMouse.fb_draw(&fb, &mouse.current, CURSOR_COLOR);

				/*
				if (event_handler[ie.type] == EV_REL)
					cursor(&ie, &mouse);
				elseif(event_handler[ie.type] == EV_KEY)
					button(&ie, &mouse);
				*/
			}

			now = rt_timer_read();
			float_dt_dmouse = ((float)(long)(p3 - p1))*1e-3; 		// us
			dmouse_time = (long) now - previous;

			if ( dmouse_worst_time<dmouse_time )
				dmouse_worst_time=dmouse_time;

			if( dmouse_time > (unsigned long)devmouse_cycle_ns )
			{
				dmouse_fault_count++;
				dmouse_worst_time=0;
			}
		}
		else
		{
			dmouse_worst_time = 0;
			dmouse_time = 0;
		}

		p1 = p3;
		p3 = rt_timer_read();
	}
}

void print_run(void *arg)
{
	long stick=0;
	unsigned int reset_count=0;
	int count=0;

	rt_printf("\nPlease WAIT at least %i (s) until the system getting ready...\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	RTIME PrintPeriod = 5e8;
	rt_task_set_periodic(NULL, TM_NOW, PrintPeriod);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle

		if ( ++count >= roundl(NSEC_PER_SEC/PrintPeriod) )
		{
			++stick;
			reset_count++;
			count=0;
		}

		if ( system_ready )
		{
			rt_printf("Time=%0.2fs\n", double_gt);
#if defined(_ECAT_ON_)
			rt_printf("# Eact actTask_dt= %lius, desTask_dt=%0.1fus, Worst_dt= %lius, Fault=%d\n",
					ethercat_time/1000, float_dt, worst_time/1000, fault_count);
#endif

#if defined(_RS232_ON_)
			rt_printf("# RS232 actTask_dt= %lius, desTask_dt=%0.1fus, Worst_dt= %lius, Fault=%d\n",
					serial_time/1000, float_dt_serial, serial_worst_time/1000, serial_fault_count);
#endif

#if defined(_DEV_MOUSE_ON_)
			rt_printf("# dMouse actTask_dt= %lius, desTask_dt=%0.1fus, Worst_dt= %lius, Fault=%d\n",
					dmouse_time/1000, float_dt_dmouse, dmouse_worst_time/1000, dmouse_fault_count);
#endif

			for(int j=0; j<ELMO_TOTAL; ++j)
			{
				rt_printf("\t \nID: %d,", j+1);

#if defined(_DEBUG_)
				//rt_printf(" StatWord: 0x%04X, ",	StatusWord[j]);
				//rt_printf(" DeviceState: %d, ",	DeviceState[j]);
				rt_printf(" ModeOfOp: %d,",		ModeOfOperationDisplay[j]);
				rt_printf("\n");
#endif
				rt_printf("\tActPos(Deg): %0.2lf,", 	ActualPos_Rad[j]*RADtoDEG);
				rt_printf("\tTarPos(Deg): %0.2lf,",	TargetPos_Rad[j]*RADtoDEG);
				rt_printf("\tActPos(inc): %d,", 	ActualPos[j]);
				//rt_printf("\tTarPos(inc): %d,", 	TargetPos[j]);
				rt_printf("\n");
				rt_printf("\tActVel(Deg/s): %0.1lf,", 	ActualVel_Rad[j]*RADtoDEG);
				rt_printf("\tTarVel(Deg/s): %0.1lf,",	TargetVel_Rad[j]*RADtoDEG);
				//rt_printf("\tActVel(inc/s): %d,", 	ActualVel[j]);
				rt_printf("\n");
				rt_printf("\tActTor(%): %d,",		ActualTor[j]);
				rt_printf("\tCtrlTor(Nm): %0.1lf", 	TargetToq[j]);
				//rt_printf("\tTarTor(%): %d", 		TargetTor[j]);
				//rt_printf("\n");
			}

#if defined(_RS232_ON_)
			rt_printf("\t \nID: %d,", 1);
			rt_printf(" Send: %d, Recieved: %d, Send: %c, Recieved: %c", kchr, chr, kchr, chr);
#endif

			rt_printf("\nForward Kinematics -->");
			for(int cNum = 0; cNum < NumChain; cNum++)
			{
				rt_printf("\n Num:%d: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.3lf, v:%0.3lf, w:%0.3lf ",cNum, ForwardPos[cNum](0), ForwardPos[cNum](1), ForwardPos[cNum](2),
						ForwardOri[cNum](0)*RADtoDEG, ForwardOri[cNum](1)*RADtoDEG, ForwardOri[cNum](2)*RADtoDEG);
				rt_printf("\n Manipulability: Task:%0.2lf, Orient:%0.2lf", TaskCondNumber[cNum], OrientCondNumber[cNum]);
				rt_printf("\n");
			}

			rt_printf("\n\n");
		}
		else
		{
			if ( count==0 )
			{
				rt_printf("\nReady Time: %i sec", stick);
				rt_printf("\nMaster State: %s, AL state: 0x%02X, ConnectedSlaves : %d",
						ecatmaster.GetEcatMasterLinkState().c_str(), ecatmaster.GetEcatMasterState(), ecatmaster.GetConnectedSlaves());
#if defined(_ECAT_ON_)
				if(ecatmaster.GetConnectedSlaves() == 0)
				{
					reset_count++;

					if(reset_count >= 10)
					{
						rt_printf("\n\nEcat master does not find any SLAVES!\n\n");
						raise(SIGINT);
					}
				}
				else
				{

					for(int i=0; i<((int)ecatmaster.GetConnectedSlaves()); i++)
					{
						rt_printf("\nID: %d , SlaveState: 0x%02X, SlaveConnection: %s, SlaveNMT: %s ", i,
								ecatmaster.GetSlaveState(i), ecatmaster.GetSlaveConnected(i).c_str(), ecatmaster.GetSlaveNMT(i).c_str());

						rt_printf(" SlaveStatus : %s,", ecat_elmo[i].GetDevState().c_str());
						rt_printf(" StatWord: 0x%04X, ", ecat_elmo[i].status_word_);

					}
					reset_count=0;

				}
#endif
				rt_printf("\n");
			}
		}
	}
}

void plot_run(void *arg)
{
	ssize_t len;
	void *msg;
	LOGGING_PACK logBuff;
	memset(&logBuff, 0, sizeof(LOGGING_PACK));

	int err = rt_queue_bind(&msg_plot, "PLOT_QUEUE", TM_INFINITE);
	if(err)
	{
		fprintf(stderr, "Failed to queue bind, code %d\n", err);
	}

	rt_task_set_periodic(NULL, TM_NOW, 1e7);

	while (1)
	{
		rt_task_wait_period(NULL);

		if ( (len = rt_queue_receive(&msg_plot, &msg, TM_INFINITE)) > 0 )
		{
			memcpy(&logBuff, msg, sizeof(LOGGING_PACK));

			datasocket.updateControlData( logBuff.ActualPos, logBuff.DesiredPos, logBuff.ActualVel,
											logBuff.DesiredVel, logBuff.ActualToq, logBuff.DesiredToq );
			datasocket.update( logBuff.Time );

			rt_queue_free(&msg_plot, msg);
		}
	}
}

void tcpip_run(void *arg)
{
	auto tcp_port = "4676";
	char buffer[256] = {0};
	char szSendMessage[256] = {0};
	int tcp_flag=0;
	Poco::Net::SocketAddress server_addr(tcp_port);
	Poco::Net::ServerSocket server_sock(server_addr);

	Poco::Net::Socket::SocketList connectedSockList;
	connectedSockList.push_back(server_sock);

	rt_task_set_periodic(NULL, TM_NOW, 1e6);

	while(1)
	{
		rt_task_wait_period(NULL);

	}
}

/****************************************************************************/
static void signal_handler(int signum)
{
	rt_printf("\n-- Signal Interrupt: %d", signum);

	rt_printf("\n-- ConsolPrint RTTask Closing....");
	rt_task_delete(&print_task);
	rt_printf("\n-- ConsolPrint RTTask Closing Success....");

#if defined(_DEV_MOUSE_ON_)
	rt_printf("\n-- devMouse RTTask Closing....");
	rt_task_delete(&devmouse_task);
	dMouse.Deactivate();
#endif

#if defined(_PLOT_ON_)
	rt_queue_unbind(&msg_plot);
	rt_printf("\n-- Plotting RTTask Closing....");
	rt_task_delete(&plot_task);
#endif

#if defined(_TCP_ON_)
	rt_printf("\n-- TCPIP RTTask Closing....");
	rt_task_delete(&tcpip_task);
	rt_printf("\n-- TCPIP RTTask Closing Success....");
#endif

#if defined(_CAN_ON_)
	rt_printf("\n-- SERCAN RTTask Closing....");
	rt_task_delete(&can_task);
	rt_task_delete(&can_rev_task);
	rt_queue_unbind(&msg_can);
	if(sercan_fd > 0)
	{
		if(close(sercan_fd)== -1)
		{
			 fprintf(stderr, "\n-- SERCAN RTTask Closing Fail, errno:%d, %s\n", errno, strerror(errno));
		}
		else
		{
			rt_printf("\n-- SERCAN RTTask Closing Success...");
		}
	}
#endif

#if defined(_RS232_ON_)
	rt_printf("\n-- Serial RTTask Closing...");
	rt_task_delete(&serial_task);
	if(serial_fd > 0)
	{
		if(close(serial_fd) == -1)
		{
			 fprintf(stderr, "\n-- Serial RTTask Closing Fail, errno:%d, %s\n", errno, strerror(errno));
		}
		else
		{
			rt_printf("\n-- Serial RTTask Closing Success...");
		}
	}
#endif

#if defined(_ECAT_ON_)
	rt_printf("\n-- EtherCAT RTTask Closing....");
	rt_task_delete(&RTArm_task);
	rt_printf("\n-- EtherCAT RTTask Closing Success....");

	ecatmaster.deactivate();
#endif

	rt_printf("\n\n\t !!RT Arm Client System Stopped!! \n");
	exit(signum);
}

/****************************************************************************/
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
	rt_print_auto_init(1);

	signal(SIGHUP, 	signal_handler);
	signal(SIGINT, 	signal_handler);
	signal(SIGQUIT, signal_handler);
	signal(SIGFPE, 	signal_handler);
	signal(SIGKILL, signal_handler);
	signal(SIGTERM, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	rt_printf("\n-- Now Configure the Devices...");

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	//cycle_ns = 200000; // nanosecond -> 5kHz
	//cycle_ns = 250000; // nanosecond -> 4kHz
	//cycle_ns = 500000; // nanosecond -> 2kHz
	cycle_ns = 750000; // nanosecond ->
	//cycle_ns = 1000000; // nanosecond -> 1kHz
	//cycle_ns = 1250000; // nanosecond -> 800Hz
	period = ((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

#if defined(_RS232_ON_)
	rt_printf("\n-- Serial Configuration");
	char PortName[30];
	unsigned int SerialBoud = 115200;
	strcpy(PortName, COM1);
	serial_fd = tp_open_serial_port(PortName, SerialBoud);
	if(serial_fd < 0)
	{
		fprintf(stderr, "\n-- Serial Closing Fail, errno:%d, %s", errno, strerror(errno));
		//perror("Serial");
		signal_handler(EXIT_FAILURE);
	}
	rt_printf("\n-- Serial Configuration : port : #%s, boudrate : %i bps", PortName, SerialBoud);
#endif

#if defined(_ECAT_ON_)

	if(ecatmaster.GetConnectedSlaves() < ELMO_TOTAL)
	{
		rt_printf("\nError: Check Ethercat slave connection!\n");
		return 1;
	}

	for(int SlaveNum=0; SlaveNum < ELMO_TOTAL; SlaveNum++)
	{
		ecat_elmo[SlaveNum].setHomingParam(hominginfo[SlaveNum].HomingOffset, hominginfo[SlaveNum].HomingMethod,
				hominginfo[SlaveNum].HomingSpeed, hominginfo[SlaveNum].HomingCurrentLimit);
		ecatmaster.addSlaveWithHoming(0, SlaveNum, &ecat_elmo[SlaveNum]);
	}

#if defined(_USE_DC_MODE_)
	ecatmaster.activateWithDC(0, cycle_ns);
#else
	ecatmaster.activate();
#endif

#endif

#if defined(_PLOT_ON_)
	rt_printf("\n-- TCP(Plot) Configuration");
	// TO DO: Create data socket server
	datasocket.setPeriod(period);

	if (datasocket.startServer(SOCK_TCP, NRMK_PORT_DATA))
		rt_printf("\n-- Data server started at IP of : %s on Port: %d", datasocket.getAddress(), NRMK_PORT_DATA);

	rt_printf("\n-- Waiting for Data Scope to connect...\n");
	datasocket.waitForConnection(0);
#endif

	// RTArm_task: create and start
	rt_printf("\n-- Now running rt task ...\n");


#if defined(_PLOT_ON_)
	rt_queue_create(&msg_plot, "PLOT_QUEUE", sizeof(LOGGING_PACK)*20, 20, Q_FIFO|Q_SHARED);

	rt_task_create(&plot_task, "PLOT_PROC_Task", 0, 80, T_FPU);
	rt_task_start(&plot_task, &plot_run, NULL);
#endif

#if defined(_ECAT_ON_)
	rt_task_create(&RTArm_task, "CONTROL_PROC_Task", 1024*1024*4, 99, T_FPU); // MUST SET at least 4MB stack-size (MAXIMUM Stack-size ; 8192 kbytes)
	rt_task_start(&RTArm_task, &RTRArm_run, NULL);
#endif

#if defined(_RS232_ON_)
	rt_task_create(&serial_task, "SERIAL_PROC_TASK", 0, 95, 0);
	rt_task_start(&serial_task, &serial_task_proc, NULL);
#endif

	rt_task_create(&print_task, "CONSOLE_PROC_Task", 0, 60, T_FPU);
	rt_task_start(&print_task, &print_run, NULL);

#if defined(_DEV_MOUSE_ON_)
	rt_task_create(&devmouse_task, "DEVMOUSE_PROC_TASK", 0, 70, 0);
	rt_task_start(&devmouse_task, &devmouse_task_proc, NULL);
#endif

#if defined(_TCP_ON_)
	rt_task_create(&tcpip_task, "TCPIP_PROC_Task", 0, 80, T_FPU);
	rt_task_start(&tcpip_task, &tcpip_run, NULL);
#endif
	// Must pause here
	pause();

	// Finalize
	signal_handler(SIGTERM);

    return 0;
}
