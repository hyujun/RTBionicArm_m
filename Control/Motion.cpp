/*
 * Motion.cpp
 *
 *  Created on: 2019. 8. 1.
 *      Author: Administrator
 */

#include "Motion.h"

namespace HYUControl {

Motion::Motion() {

	this->pManipulator = NULL;
	TotalDoF=0;
	TotalChain=0;
	MotionProcess=0;
}

Motion::Motion(SerialManipulator *_pManipulator)
{
	this->pManipulator = _pManipulator;

	TotalDoF = pManipulator->GetTotalDoF();
	TotalChain = pManipulator->GetTotalChain();

	MotionProcess=0;
	motion_cycle = 0;

	dq.resize(TotalDoF);
	dqdot.resize(TotalDoF);
	dqddot.resize(TotalDoF);

	TargetPos.resize(TotalDoF);
	TargetPosTask.resize(7, TotalChain);
}

Motion::~Motion() {

}

uint16_t Motion::JointMotion(double *_dq, double *_dqdot, double *_dqddot, VectorXd &_Target, double *_q, double *_qdot, double &_Time, uint16_t &_StatusWord, uint16_t &_MotionType)
{
	this->MotionCommand = _MotionType;

	if(_Time >= 1.5 && _StatusWord == SYSTEM_BEGIN)
	{
		MotionCommand = MOVE_ZERO;
		//MotionCommand = MOVE_CLIK_JOINT;
		_MotionType = MotionCommand;

	}

	else if( MotionCommand == MOVE_ZERO && MotionProcess == MOVE_ZERO && _StatusWord == TARGET_ACHIEVED )
		{
			if(motion_cycle == 0)
			{
				//MotionCommand = MOVE_ZERO;
				//MotionCommand = MOVE_CUSTOMIZE;
				MotionCommand = MOVE_JOB;
			}
			else if(motion_cycle == 1)
			{
				MotionCommand = MOVE_CUSTOMIZE4;
			}
			else if(motion_cycle == 2)
			{
				MotionCommand = MOVE_CUSTOMIZE8;
			}
			else if(motion_cycle == 3)
			{
				MotionCommand = MOVE_CUSTOMIZE15;
			}
			_MotionType = MotionCommand;
		}

	else if( MotionCommand == MOVE_JOB && MotionProcess == MOVE_JOB && _StatusWord == TARGET_ACHIEVED )
	{
		if(motion_cycle == 0)
		{
			//MotionCommand = MOVE_ZERO;
			MotionCommand = MOVE_CUSTOMIZE;
			//MotionCommand = MOVE_JOB;
		}
		else
		{
			MotionCommand = MOVE_ZERO;
			//MotionCommand = MOVE_CUSTOMIZE;
			//MotionCommand = MOVE_JOB;
		}

		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_CUSTOMIZE && MotionProcess == MOVE_CUSTOMIZE && _StatusWord == TARGET_ACHIEVED )
	{
		if(motion_cycle == 0)
		{
			MotionCommand = MOVE_ZERO;
			//MotionCommand = MOVE_CUSTOMIZE1;
			//MotionCommand = MOVE_JOB;
			motion_cycle = 1;
		}
		else
		{
			//MotionCommand = MOVE_ZERO;
			//MotionCommand = MOVE_CUSTOMIZE;
			MotionCommand = MOVE_JOB;
		}
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_CUSTOMIZE4 && MotionProcess == MOVE_CUSTOMIZE4 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE5;;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE5 && MotionProcess == MOVE_CUSTOMIZE5 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE6;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE6 && MotionProcess == MOVE_CUSTOMIZE6 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE7;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE7 && MotionProcess == MOVE_CUSTOMIZE7 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_ZERO;
			motion_cycle = 2;
			//motion_cycle = 0;
			_MotionType = MotionCommand;
		}



	else if( MotionCommand == MOVE_CUSTOMIZE8 && MotionProcess == MOVE_CUSTOMIZE8 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE9;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE9 && MotionProcess == MOVE_CUSTOMIZE9 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE10;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE10 && MotionProcess == MOVE_CUSTOMIZE10 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE11;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE11 && MotionProcess == MOVE_CUSTOMIZE11 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE12;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE12 && MotionProcess == MOVE_CUSTOMIZE12 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE13;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE13 && MotionProcess == MOVE_CUSTOMIZE13 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_ZERO;
			motion_cycle = 0;
			//MotionCommand = MOVE_CUSTOMIZE14;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE14 && MotionProcess == MOVE_CUSTOMIZE14 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_ZERO;
			motion_cycle = 0;
			_MotionType = MotionCommand;
		}


	else if( MotionCommand == MOVE_CUSTOMIZE15 && MotionProcess == MOVE_CUSTOMIZE15 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE16;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE16 && MotionProcess == MOVE_CUSTOMIZE16 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE17;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE17 && MotionProcess == MOVE_CUSTOMIZE17 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE18;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE18 && MotionProcess == MOVE_CUSTOMIZE18 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE19;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE19 && MotionProcess == MOVE_CUSTOMIZE19 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE20;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE20 && MotionProcess == MOVE_CUSTOMIZE20 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE21;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE21 && MotionProcess == MOVE_CUSTOMIZE21 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_CUSTOMIZE22;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE22 && MotionProcess == MOVE_CUSTOMIZE22 && _StatusWord == TARGET_ACHIEVED)
		{
			//MotionCommand = MOVE_CUSTOMIZE23;
			MotionCommand = MOVE_ZERO;
			_MotionType = MotionCommand;
		}
	else if( MotionCommand == MOVE_CUSTOMIZE23 && MotionProcess == MOVE_CUSTOMIZE23 && _StatusWord == TARGET_ACHIEVED)
		{
			MotionCommand = MOVE_ZERO;
			_MotionType = MotionCommand;
		}


	else if( MotionCommand == MOVE_JOINT_CYCLIC && MotionProcess == MOVE_JOINT_CYCLIC && (_Time >= MotionInitTime+20.0))
	{
		MotionCommand = MOVE_JOB;
		_MotionType = MotionCommand;
	}

	q = Map<VectorXd>(_q, pManipulator->GetTotalDoF());
	qdot = Map<VectorXd>(_qdot, pManipulator->GetTotalDoF());

	dq = Map<VectorXd>(_dq, pManipulator->GetTotalDoF());
	dqdot = Map<VectorXd>(_dqdot, pManipulator->GetTotalDoF());
	dqddot = Map<VectorXd>(_dqddot, pManipulator->GetTotalDoF());

	if( MotionCommand == MOVE_ZERO ) //home posture
	{
		if( MotionCommand == MotionCommand_p )
		{
			if(JointPoly5th.isReady() == 0 && NewTarget==1)
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_ZERO;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(3) = 10.0*DEGtoRAD;
			_Target = TargetPos;

			TrajectoryTime=4;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_JOB ) //job posture
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}


			MotionProcess = MOVE_JOB;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 90.0*DEGtoRAD;
			TargetPos(1) = -30.0*DEGtoRAD;
			TargetPos(2) = -70.0*DEGtoRAD;
			TargetPos(3) = 110.0*DEGtoRAD;
			TargetPos(4) = -30.0*DEGtoRAD;

			//TargetPos(0) = -40.0*DEGtoRAD;
			//TargetPos(1) = 20.0*DEGtoRAD;
			//TargetPos(2) = 20.0*DEGtoRAD;
			//TargetPos(3) = 80.0*DEGtoRAD;
			//TargetPos(4) = 0.0*DEGtoRAD;
			//TargetPos(5) = 0.0*DEGtoRAD;
			_Target = TargetPos;

			TrajectoryTime=4;
			NewTarget=1;
			MotionInitTime = _Time;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE ) //custom
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 20.0*DEGtoRAD;
			TargetPos(1) = -90.0*DEGtoRAD;
			TargetPos(2) = -20.0*DEGtoRAD;
			TargetPos(3) = 20.0*DEGtoRAD;
			TargetPos(4) = 25.0*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=4;
			NewTarget=1;
			MotionInitTime = _Time;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE1) //bow
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE1;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 7.67*DEGtoRAD;
			TargetPos(1) = -26.27*DEGtoRAD;
			TargetPos(2) = -60.44*DEGtoRAD;
			TargetPos(3) = 104.10*DEGtoRAD;
			TargetPos(4) = -17.73*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE2) //gym1
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE2;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 20.0*DEGtoRAD;
			TargetPos(1) = -5.0*DEGtoRAD;
			TargetPos(2) = -20.0*DEGtoRAD;
			TargetPos(3) = 10.0*DEGtoRAD;
			TargetPos(4) = -25.0*DEGtoRAD;
			TargetPos(5) = 0.0;

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE3) //gym2
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE3;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 20.0*DEGtoRAD;
			TargetPos(1) = -5.0*DEGtoRAD;
			TargetPos(2) = -20.0*DEGtoRAD;
			TargetPos(3) = 10.0*DEGtoRAD;
			TargetPos(4) = -25.0*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE4)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE4;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 23.77*DEGtoRAD;
			TargetPos(1) = -3.92*DEGtoRAD;
			TargetPos(2) = -39.17*DEGtoRAD;
			TargetPos(3) = 110.71*DEGtoRAD;
			TargetPos(4) = -35.58*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=2.2;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE5)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE5;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 55.31*DEGtoRAD;
			TargetPos(1) = -27.12*DEGtoRAD;
			TargetPos(2) = -50.98*DEGtoRAD;
			TargetPos(3) = 30.25*DEGtoRAD;
			TargetPos(4) = -10.01*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=2.2;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE6)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE6;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 100.0*DEGtoRAD;
			TargetPos(1) = -30.0*DEGtoRAD;
			TargetPos(2) = -70.0*DEGtoRAD;
			TargetPos(3) = 110.0*DEGtoRAD;
			TargetPos(4) = -40.0*DEGtoRAD;


			_Target = TargetPos;

			TrajectoryTime=2.2;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE7)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE7;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 20.0*DEGtoRAD;
			TargetPos(1) = -90.0*DEGtoRAD;
			TargetPos(2) = -20.0*DEGtoRAD;
			TargetPos(3) = 20.0*DEGtoRAD;
			TargetPos(4) = 30.0*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=2.2;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE8)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE8;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 11.79*DEGtoRAD;
			TargetPos(1) = -1.10*DEGtoRAD;
			TargetPos(2) = -9.96*DEGtoRAD;
			TargetPos(3) = 114.45*DEGtoRAD;
			TargetPos(4) = -4.11*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=2;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE9)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE9;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 148.46*DEGtoRAD;
			TargetPos(1) = -47.94*DEGtoRAD;
			TargetPos(2) = -52.99*DEGtoRAD;
			TargetPos(3) = 82.73*DEGtoRAD;
			TargetPos(4) = 1.63*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=2;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE10)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE10;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 155.93*DEGtoRAD;
			TargetPos(1) = -4.05*DEGtoRAD;
			TargetPos(2) = -52.99*DEGtoRAD;
			TargetPos(3) = 42.18*DEGtoRAD;
			TargetPos(4) = 2.21*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=2;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE11)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE11;
		}
		else
		{
			TargetPos.setZero();
			TargetPos.setZero();
			TargetPos(0) = 148.46*DEGtoRAD;
			TargetPos(1) = -47.94*DEGtoRAD;
			TargetPos(2) = -52.99*DEGtoRAD;
			TargetPos(3) = 82.73*DEGtoRAD;
			TargetPos(4) = 1.63*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=1;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE12)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE12;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 155.93*DEGtoRAD;
			TargetPos(1) = -4.05*DEGtoRAD;
			TargetPos(2) = -52.99*DEGtoRAD;
			TargetPos(3) = 42.18*DEGtoRAD;
			TargetPos(4) = 2.21*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=1;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE13)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE13;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 148.46*DEGtoRAD;
			TargetPos(1) = -47.94*DEGtoRAD;
			TargetPos(2) = -52.99*DEGtoRAD;
			TargetPos(3) = 82.73*DEGtoRAD;
			TargetPos(4) = 1.63*DEGtoRAD;


			_Target = TargetPos;

			TrajectoryTime=1;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE14)
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				if(JointPoly5th.isFinished() == 1)
					_StatusWord = TARGET_ACHIEVED;
			}

			MotionProcess = MOVE_CUSTOMIZE14;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 11.79*DEGtoRAD;
			TargetPos(1) = -1.10*DEGtoRAD;
			TargetPos(2) = -9.96*DEGtoRAD;
			TargetPos(3) = 114.45*DEGtoRAD;
			TargetPos(4) = -4.11*DEGtoRAD;

			_Target = TargetPos;

			TrajectoryTime=1;
			NewTarget=1;
			_StatusWord = TARGET_MOVING;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE15)
		{
			if( MotionCommand == MotionCommand_p )
			{
				if( JointPoly5th.isReady() == 0 && NewTarget==1 )
				{
					JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
					NewTarget=0;
				}
				else
				{
					JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
					if(JointPoly5th.isFinished() == 1)
						_StatusWord = TARGET_ACHIEVED;
				}

				MotionProcess = MOVE_CUSTOMIZE15;
			}
			else
			{
				TargetPos.setZero();
				TargetPos(0) = 3.27*DEGtoRAD;
				TargetPos(1) = -0.04*DEGtoRAD;
				TargetPos(2) = 0.01*DEGtoRAD;
				TargetPos(3) = 92.84*DEGtoRAD;
				TargetPos(4) = -7.73*DEGtoRAD;


				_Target = TargetPos;

				TrajectoryTime=4;
				NewTarget=1;
				_StatusWord = TARGET_MOVING;
			}
		}
	else if( MotionCommand == MOVE_CUSTOMIZE16)
		{
			if( MotionCommand == MotionCommand_p )
			{
				if( JointPoly5th.isReady() == 0 && NewTarget==1 )
				{
					JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
					NewTarget=0;
				}
				else
				{
					JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
					if(JointPoly5th.isFinished() == 1)
						_StatusWord = TARGET_ACHIEVED;
				}

				MotionProcess = MOVE_CUSTOMIZE16;
			}
			else
			{
				TargetPos.setZero();
				TargetPos(0) = 47.66*DEGtoRAD;
				TargetPos(1) = -0.03*DEGtoRAD;
				TargetPos(2) = 0.01*DEGtoRAD;
				TargetPos(3) = 43.99*DEGtoRAD;
				TargetPos(4) = -7.81*DEGtoRAD;


				_Target = TargetPos;

				TrajectoryTime=4;
				NewTarget=1;
				_StatusWord = TARGET_MOVING;
			}
		}
	else if( MotionCommand == MOVE_CUSTOMIZE17)
		{
			if( MotionCommand == MotionCommand_p )
			{
				if( JointPoly5th.isReady() == 0 && NewTarget==1 )
				{
					JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
					NewTarget=0;
				}
				else
				{
					JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
					if(JointPoly5th.isFinished() == 1)
						_StatusWord = TARGET_ACHIEVED;
				}

				MotionProcess = MOVE_CUSTOMIZE17;
			}
			else
			{
				TargetPos.setZero();
				TargetPos(0) = 44.12*DEGtoRAD;
				TargetPos(1) = -0.35*DEGtoRAD;
				TargetPos(2) = 0.01*DEGtoRAD;
				TargetPos(3) = 119.56*DEGtoRAD;
				TargetPos(4) = -19.39*DEGtoRAD;


				_Target = TargetPos;

				TrajectoryTime=4;
				NewTarget=1;
				_StatusWord = TARGET_MOVING;
			}
		}
	else if( MotionCommand == MOVE_CUSTOMIZE18)
		{
			if( MotionCommand == MotionCommand_p )
			{
				if( JointPoly5th.isReady() == 0 && NewTarget==1 )
				{
					JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
					NewTarget=0;
				}
				else
				{
					JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
					if(JointPoly5th.isFinished() == 1)
						_StatusWord = TARGET_ACHIEVED;
				}

				MotionProcess = MOVE_CUSTOMIZE18;
			}
			else
			{
				TargetPos.setZero();
				TargetPos(0) = 3.27*DEGtoRAD;
				TargetPos(1) = -0.04*DEGtoRAD;
				TargetPos(2) = 0.01*DEGtoRAD;
				TargetPos(3) = 92.84*DEGtoRAD;
				TargetPos(4) = -7.73*DEGtoRAD;


				_Target = TargetPos;

				TrajectoryTime=4;
				NewTarget=1;
				_StatusWord = TARGET_MOVING;
			}
		}
	else if( MotionCommand == MOVE_CUSTOMIZE19)
		{
			if( MotionCommand == MotionCommand_p )
			{
				if( JointPoly5th.isReady() == 0 && NewTarget==1 )
				{
					JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
					NewTarget=0;
				}
				else
				{
					JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
					if(JointPoly5th.isFinished() == 1)
						_StatusWord = TARGET_ACHIEVED;
				}

				MotionProcess = MOVE_CUSTOMIZE19;
			}
			else
			{
				TargetPos.setZero();
				TargetPos(0) = 2.30*DEGtoRAD;
				TargetPos(1) = 1.35*DEGtoRAD;
				TargetPos(2) = -49.97*DEGtoRAD;
				TargetPos(3) = 96.21*DEGtoRAD;
				TargetPos(4) = -23.07*DEGtoRAD;


				_Target = TargetPos;

				TrajectoryTime=4;
				NewTarget=1;
				_StatusWord = TARGET_MOVING;
			}
		}
	else if( MotionCommand == MOVE_CUSTOMIZE20)
		{
			if( MotionCommand == MotionCommand_p )
			{
				if( JointPoly5th.isReady() == 0 && NewTarget==1 )
				{
					JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
					NewTarget=0;
				}
				else
				{
					JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
					if(JointPoly5th.isFinished() == 1)
						_StatusWord = TARGET_ACHIEVED;
				}

				MotionProcess = MOVE_CUSTOMIZE20;
			}
			else
			{
				TargetPos.setZero();
				TargetPos(0) = 2.62*DEGtoRAD;
				TargetPos(1) = -0.27*DEGtoRAD;
				TargetPos(2) = -29.66*DEGtoRAD;
				TargetPos(3) = 93.16*DEGtoRAD;
				TargetPos(4) = -22.83*DEGtoRAD;


				_Target = TargetPos;

				TrajectoryTime=4;
				NewTarget=1;
				_StatusWord = TARGET_MOVING;
			}
		}
	else if( MotionCommand == MOVE_CUSTOMIZE21)
		{
			if( MotionCommand == MotionCommand_p )
			{
				if( JointPoly5th.isReady() == 0 && NewTarget==1 )
				{
					JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
					NewTarget=0;
				}
				else
				{
					JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
					if(JointPoly5th.isFinished() == 1)
						_StatusWord = TARGET_ACHIEVED;
				}

				MotionProcess = MOVE_CUSTOMIZE21;
			}
			else
			{
				TargetPos.setZero();
				TargetPos(0) = 3.27*DEGtoRAD;
				TargetPos(1) = -0.04*DEGtoRAD;
				TargetPos(2) = -0.01*DEGtoRAD;
				TargetPos(3) = 92.84*DEGtoRAD;
				TargetPos(4) = -7.73*DEGtoRAD;


				_Target = TargetPos;

				TrajectoryTime=4;
				NewTarget=1;
				_StatusWord = TARGET_MOVING;
			}
		}
	else if( MotionCommand == MOVE_CUSTOMIZE22)
		{
			if( MotionCommand == MotionCommand_p )
			{
				if( JointPoly5th.isReady() == 0 && NewTarget==1 )
				{
					JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
					NewTarget=0;
				}
				else
				{
					JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
					if(JointPoly5th.isFinished() == 1)
						_StatusWord = TARGET_ACHIEVED;
				}

				MotionProcess = MOVE_CUSTOMIZE22;
			}
			else
			{
				TargetPos.setZero();
				TargetPos(0) = 120.26*DEGtoRAD;
				TargetPos(1) = -14.30*DEGtoRAD;
				TargetPos(2) = -14.70*DEGtoRAD;
				TargetPos(3) = 59.97*DEGtoRAD;
				TargetPos(4) = -22.03*DEGtoRAD;


				_Target = TargetPos;

				TrajectoryTime=4;
				NewTarget=1;
				_StatusWord = TARGET_MOVING;
			}
		}
	else if( MotionCommand == MOVE_CUSTOMIZE23)
		{
			if( MotionCommand == MotionCommand_p )
			{
				if( JointPoly5th.isReady() == 0 && NewTarget==1 )
				{
					JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
					NewTarget=0;
				}
				else
				{
					JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
					if(JointPoly5th.isFinished() == 1)
						_StatusWord = TARGET_ACHIEVED;
				}

				MotionProcess = MOVE_CUSTOMIZE23;
			}
			else
			{
				TargetPos.setZero();
				TargetPos(0) = 44.55*DEGtoRAD;
				TargetPos(1) = -0.83*DEGtoRAD;
				TargetPos(2) = -2.98*DEGtoRAD;
				TargetPos(3) = 46.08*DEGtoRAD;
				TargetPos(4) = -23.01*DEGtoRAD;


				_Target = TargetPos;

				TrajectoryTime=4;
				NewTarget=1;
				_StatusWord = TARGET_MOVING;
			}
		}



	else if( MotionCommand == MOVE_JOINT_CYCLIC )
	{
		if( MotionCommand != MotionCommand_p )
		{
			MotionInitTime = _Time;
		}
		else
		{
			_T = 18.0;
			_omega = 2.0*M_PI/_T;
			_amp = 70;

			dq(0) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(0) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(0) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			_T = 10.0;
			_omega = 2.0*M_PI/_T;
			_amp = 20;

			dq(1) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.8481) - 15*M_PI/180.0;
			dqdot(1) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.8481);
			dqddot(1) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.8481);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 50;

			dq(2) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.6435) - 30*M_PI/180.0;
			dqdot(2) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.6435);
			dqddot(2) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.6435);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 50;

			dq(3) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.4115) - 20*M_PI/180.0;
			dqdot(3) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.4115);
			dqddot(3) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.4115);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 45;

			dq(4) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.729) - 30*M_PI/180.0;
			dqdot(4) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.729);
			dqddot(4) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.729);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 70.0;

			dq(5) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(5) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(5) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			MotionProcess = MOVE_JOINT_CYCLIC;
		}
	}

	MotionCommand_p = MotionCommand;

	Map<VectorXd>(_dq, pManipulator->GetTotalDoF()) = dq;
	Map<VectorXd>(_dqdot, pManipulator->GetTotalDoF()) = dqdot;
	Map<VectorXd>(_dqddot, pManipulator->GetTotalDoF()) = dqddot;

	return MotionProcess;
}

uint16_t Motion::TaskMotion( VectorXd *_dx, VectorXd *_dxdot, VectorXd *_dxddot, double &_Time, uint16_t &_StatusWord, uint16_t &_MotionType )
{
	MotionCommandTask = _MotionType;

	//int TotalTask = 4*TotalChain;

	if( MotionCommandTask == MOVE_CLIK_JOINT )
	{
		if( MotionCommandTask == MotionCommandTask_p )
		{
			//if( TaskPoly5th.isReady()==0 && NewTarget==1 )
			//{
			//	TaskPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalTask);
			//	NewTarget=0;
			//}
			//else
			//{
			//	TaskPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
			//}

			for(int i=0; i<TotalChain; i++)
			{
				//_dx[i].resize(7);
				_dx[i].head(3) = TargetPosTask.block(0,i,3,1);
				_dx[i](3) = TargetPosTask(3,i);
				_dx[i].tail(3) = TargetPosTask.block(4,i,3,1);

				_dxdot[i].setZero();
				_dxddot[i].setZero();
			}

			MotionProcess = MOVE_CLIK_JOINT;
		}
		else
		{
			TargetPosTask.setZero();
			TargetPosTask.block(0,0,3,1) << 0, -1, 0;
			TargetPosTask.block(3,0,1,1) << 90.0*DEGtoRAD;
			TargetPosTask.block(4,0,3,1) << 0.310, -0.310, 0.420;

			TargetPosTask.block(0,1,3,1) << 0, -1, 0;
			TargetPosTask.block(3,1,1,1) << 90.0*DEGtoRAD;
			TargetPosTask.block(4,1,3,1) << 0.310, 0.310, 0.420;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}

	MotionCommandTask_p = MotionCommandTask;

	return MotionProcess;
}

} /* namespace hyuCtrl */
