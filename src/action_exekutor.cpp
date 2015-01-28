#include "exekutor/action_exekutor.h"

namespace exekutor
{

StringVector ActionExekutor::action_str_list;
ActionExekutorPtrVector ActionExekutor::action_ptr_list;
boost::shared_ptr <tf::TransformListener> ActionExekutor::tf_listener_base_;

int ActionExekutor::my_peis_id;

ActionExekutor::ActionExekutor(std::string robot_name, std::string a_name, bool listen_on_tf)
{
	if(peiskmt_isRunning() && ros::isInitialized())
	{
		ActionExekutor::my_peis_id = peiskmt_peisid();
	}

	else
	{
		if(!peiskmt_isRunning)
			printf("ERROR: PeisKernel is not running! Aborted.\n");
		else
			printf("ERROR: ROS is not running! Aborted.\n");

		exit(0);
	}

	robot_name_ = robot_name;
	action_name_ = a_name;
	tuple_set_[COMMAND] = "in." + robot_name + "." + a_name + "." + "command";
	tuple_set_[STATE] = "out." + robot_name + "." + a_name + "." + "state";
	tuple_set_[PARAMS] = "in." + robot_name + "." + a_name + "." + "parameters";
	tuple_set_[RESULT] = "out." + robot_name + "." + a_name + "." + "result";
	tuple_set_[PROGRESS] = "out." + robot_name + "." + a_name + "." + "progress";

	addAction(this);
	initiateMetaTuples();
	setState(IDLE);
	setResult("NOTHING");

	if(!ActionExekutor::tf_listener_base_ && listen_on_tf)
	{
		ROS_INFO("*****************************************************");
		ROS_INFO("Hip hip hurray! Let\'s hear it for the smart ass day!");
		ROS_INFO("*****************************************************");

		tf_listener_base_ = boost::shared_ptr <tf::TransformListener> (new tf::TransformListener(ros::Duration(10)));
	}

	if(listen_on_tf)
		tf_listener_ = boost::shared_ptr <tf::TransformListener> (tf_listener_base_);
}

ActionExekutor::~ActionExekutor()
{
}

void ActionExekutor::printAll()
{
	ROS_INFO("ActionStrList has these: ");
	for(std::vector <std::string>::iterator iter_str = action_str_list.begin(); iter_str != action_str_list.end(); iter_str++)
	{
		ROS_INFO("\n%s", iter_str->c_str());
	}
	ROS_INFO("\n");
	
	ROS_INFO("ActionExekutorPtr has these: ");
	
	for(std::vector <ActionExekutor*>::iterator iter_actptr = action_ptr_list.begin(); iter_actptr != action_ptr_list.end(); iter_actptr)
	{
		ROS_INFO("\n%s", (*iter_actptr)->action_name_.c_str());
	}
}

std::vector<double> ActionExekutor::extractParams(const char p[])
{
	char *copyOfString = new char[strlen(p) + 1];
	strcpy(copyOfString, p);

	std::vector<double> double_values;

	char *pch;
	int cmd_args = 0;

	pch = strtok (copyOfString," ,");
	while (pch != NULL)
	{
		cmd_args++;
		if(!isalpha(pch[0]))
			double_values.push_back(atof(pch));
		pch = strtok (NULL, " ,");
	}

	delete copyOfString;
	return double_values;
}

std::vector<std::string> ActionExekutor::extractParamStrings(const char p[])
{
	char *copyOfString = new char[strlen(p) + 1];
	strcpy(copyOfString, p);

	std::vector<std::string> _values;

	char *pch;
	int cmd_args = 0;

	pch = strtok (copyOfString," ,");
	while (pch != NULL)
	{
		cmd_args++;
		_values.push_back(pch);
		pch = strtok (NULL, " ,");
	}

	delete copyOfString;
	return _values;
}

void ActionExekutor::initiateMetaTuples()
{

	printf("[*ActionExekutor*] Creating Meta Tuples %s and %s.\n",
			tuple_set_[COMMAND].c_str(),
			tuple_set_[PARAMS].c_str());

	/* One tuple each for Command, Parameters and State */
	peiskmt_declareMetaTuple(my_peis_id, tuple_set_[COMMAND].c_str());
	peiskmt_declareMetaTuple(my_peis_id, tuple_set_[PARAMS].c_str());

	peiskmt_setStringTuple(tuple_set_[PROGRESS].c_str(), "-");

	// State is no longer a meta-tuple in exekutor.
	//peiskmt_declareMetaTuple(my_peis_id, tuple_set_[STATE].c_str());
	// Result is no longer a meta-tuple in exekutor.
	//peiskmt_declareMetaTuple(my_peis_id, tuple_set_[RESULT].c_str());

	peiskmt_subscribeIndirectly(my_peis_id, tuple_set_[COMMAND].c_str());
	peiskmt_subscribeIndirectly(my_peis_id, tuple_set_[PARAMS].c_str());

}

void ActionExekutor::waitForLink()
{
	for(std::vector<ActionExekutor*>::iterator it = action_ptr_list.begin(); it!= action_ptr_list.end(); it++)
	{
		(*it)->waitForMyLink();
	}

	for(std::vector<ActionExekutor*>::iterator it = action_ptr_list.begin(); it!= action_ptr_list.end(); it++)
	{
		pthread_join((*it)->thread_id_, NULL);
	}
}

void ActionExekutor::waitForMyLink()
{
	pthread_create(&thread_id_, NULL, ActionExekutor::waitThread, (void *) this);
}

void ActionExekutor::cancelWaiting()
{
	pthread_cancel(thread_id_);
}

void* ActionExekutor::waitThread(void *_this_)
{
	while(ros::ok() && peiskmt_isRunning())
	{
		if(peiskmt_isMetaTuple( ((ActionExekutor*) _this_)->my_peis_id, ((ActionExekutor*) _this_)->tuple_set_[COMMAND].c_str()))
		{
			//ROS_INFO("%s Link Status: CONNECTED\n", ((ActionExekutor*) _this_)->action_name_.c_str());
			if(!peiskmt_isMetaTuple(((ActionExekutor*) _this_)->my_peis_id, ((ActionExekutor*) _this_)->tuple_set_[PARAMS].c_str()))
			{
				printf("\nParameter tuple was not linked. Something is not right. All tuples should be linked.\n");
 				sleep(1);
				continue;
			}

			// Typical function call.
			PeisTuple stateTup = ((ActionExekutor*) _this_)->getStateTuple();
			if(strcmp(stateTup.data, "COMPLETED") != 0 && strcmp(stateTup.data, "FAILED") != 0)
				((ActionExekutor*) _this_)->startAction();
		}

		//else
			//ROS_INFO("%s Link Status: Disconnected\n", ((ActionExekutor*) _this_)->action_name_.c_str());
			
		usleep(500000);
	}
	
	return NULL;
}

void ActionExekutor::startAction(int i)
{

		while(ros::ok() && i--)
		{
			PeisTuple recdTuple = getCommandTuple();

			if(strcmp("OFF",recdTuple.data)==0)
			{
				printf("\nCommand is OFF.\n");
				sleep(1);
				continue;
			}

			if(strcmp("ON",recdTuple.data)==0)
			{
				setState(RUNNING);
				actionThread();
				break;
			}

		}
		/*
		if(timeout_flag == true)
		{
			printf("Action timed out!\n");
			setState(FAILED);
		}
		*/
		//resetMetaTuples();
}

void ActionExekutor::resetMetaTuples()
{
	peiskmt_setMetaTuple(my_peis_id, this->tuple_set_[COMMAND].c_str(), -1, "NULL");
	peiskmt_setMetaTuple(my_peis_id, this->tuple_set_[PARAMS].c_str(), -1, "NULL");

	// State and Result are no longer meta-tuples.
}

PeisTuple ActionExekutor::getParamTuple()
{
	PeisTuple* paramTuple = peiskmt_getTupleIndirectly(my_peis_id, tuple_set_[PARAMS].c_str(), PEISK_KEEP_OLD);
	ROS_INFO("Fetching parameters for action...");
	while(!paramTuple)
	{
		paramTuple = peiskmt_getTupleIndirectly(my_peis_id, tuple_set_[PARAMS].c_str(), PEISK_KEEP_OLD);
		usleep(500000);
	}
	return *paramTuple;
}

PeisTuple ActionExekutor::getCommandTuple()
{
	PeisTuple* commandTuple = peiskmt_getTupleIndirectly(my_peis_id, tuple_set_[COMMAND].c_str(), PEISK_KEEP_OLD);
	ROS_INFO("Fetching Command for action...");
	while(!commandTuple)
	{
		commandTuple = peiskmt_getTupleIndirectly(my_peis_id, tuple_set_[COMMAND].c_str(), PEISK_KEEP_OLD);
		usleep(500000);
	}
	return *commandTuple;
}

PeisTuple ActionExekutor::getStateTuple()
{
	PeisTuple* stateTuple = peiskmt_getTuple(my_peis_id, tuple_set_[STATE].c_str(), PEISK_KEEP_OLD);
	//ROS_INFO("Fetching state.");
	while(!stateTuple)
	{
		stateTuple = peiskmt_getTuple(my_peis_id, tuple_set_[STATE].c_str(), PEISK_KEEP_OLD);
		sleep(500000);
	}
	return *stateTuple;
}
}
