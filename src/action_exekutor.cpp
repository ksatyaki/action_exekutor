/**
 * @file   action_exekutor.cpp
 * @author Chittaranjan S Srinivas
 * 
 * @brief  This file defines the ActionExekutor Base class and the members.
 *     
 * Copyright (C) 2015  Chittaranjan Srinivas Swaminathan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

#include "exekutor/action_exekutor.h"

namespace exekutor
{

StringVector ActionExekutor::action_str_list;
ActionExekutorPtrVector ActionExekutor::action_ptr_list;
boost::shared_ptr <tf::TransformListener> ActionExekutor::tf_listener_base_;

int ActionExekutor::my_peis_id;
std::map <std::string, pthread_t> ActionExekutor::action_name_to_thread_id_;
std::map <std::string, bool> ActionExekutor::action_name_to_is_running_;
int ActionExekutor::action_timeout_ = 120;

ActionExekutor::ActionExekutor(std::string robot_name, std::string a_name, bool listen_on_tf)
{
	if(peiskmt_isRunning() && ros::isInitialized())
	{
		ActionExekutor::my_peis_id = peiskmt_peisid();
	}

	else
	{
		if(!peiskmt_isRunning())
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
	
	for(std::vector <ActionExekutor*>::iterator iter_actptr = action_ptr_list.begin(); iter_actptr != action_ptr_list.end(); iter_actptr++)
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

	//peiskmt_subscribeIndirectly(my_peis_id, tuple_set_[COMMAND].c_str());
	//peiskmt_subscribeIndirectly(my_peis_id, tuple_set_[PARAMS].c_str());

}

void ActionExekutor::waitForLink()
{
	for(std::vector<ActionExekutor*>::iterator it = action_ptr_list.begin(); it!= action_ptr_list.end(); it++)
	{
		peiskmt_registerMetaTupleCallback(peisk_peisid(), (*it)->tuple_set_[COMMAND].c_str(), *it, &ActionExekutor::commandCallback);
		peiskmt_registerMetaTupleCallback(peisk_peisid(), (*it)->tuple_set_[PARAMS].c_str(), *it, &ActionExekutor::paramsCallback);
		peiskmt_registerTupleCallback(peisk_peisid(), (*it)->tuple_set_[STATE].c_str(), *it, &ActionExekutor::stateCallback);
	}

	ROS_INFO("Callbacks have been registered! Waiting for commands... ");
	while(ros::ok() && peiskmt_isRunning()) {
		sleep(1);
	}
}

void ActionExekutor::commandCallback(PeisTuple* tuple, void* _this_) {
	//ROS_INFO("CALLED BACK");
	ActionExekutor* _this_ptr_ = static_cast<ActionExekutor*>(_this_);
	if(std::string("ON").compare(tuple->data) == 0) {
		if(std::string("COMPLETED").compare(_this_ptr_->getStateTuple_st().data) != 0 && std::string("FAILED").compare(_this_ptr_->getStateTuple_st().data) != 0) {
			if(action_name_to_is_running_[_this_ptr_->action_name_]) {
				// cancel thread and run again.
				pthread_cancel(action_name_to_thread_id_[_this_ptr_->action_name_]);
				ROS_INFO("Cancelled already running exekutor thread. Spawning new one.");
			}
			
			pthread_create(&action_name_to_thread_id_[_this_ptr_->action_name_], NULL, &ActionExekutor::startAction, _this_);
		}
			
	}
}

void ActionExekutor::paramsCallback(PeisTuple* tuple, void* _this_) {
	ROS_INFO("Received new parameters: %s", tuple->data);
}

void ActionExekutor::stateCallback(PeisTuple* tuple, void* _this_) {
	//ROS_INFO("CALLED BACK");
	ActionExekutor* _this_ptr_ = static_cast<ActionExekutor*>(_this_);
	if(std::string("ON").compare(_this_ptr_->getCommandTuple_st().data) == 0 && std::string("IDLE").compare(tuple->data) == 0) {
		if(action_name_to_is_running_[_this_ptr_->action_name_]) {
			// cancel thread and run again.
			pthread_cancel(action_name_to_thread_id_[_this_ptr_->action_name_]);
			ROS_INFO("Cancelled already running exekutor thread. Spawning new one.");
		}
			
		pthread_create(&action_name_to_thread_id_[_this_ptr_->action_name_], NULL, &ActionExekutor::startAction, _this_);
		action_name_to_is_running_[_this_ptr_->action_name_] = true;
	}
}

void* ActionExekutor::startAction(void* this_)
{
	ActionExekutor* _this_ = static_cast <ActionExekutor*> (this_);
	_this_->actionThread();
	action_name_to_is_running_[_this_->action_name_] = false;
	return NULL;
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
	//ROS_INFO("Fetching parameters for action...");
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
	//ROS_INFO("Fetching Command for action...");
	while(!commandTuple)
	{
		commandTuple = peiskmt_getTupleIndirectly(my_peis_id, tuple_set_[COMMAND].c_str(), PEISK_KEEP_OLD);
		usleep(500000);
	}
	return *commandTuple;
}

PeisTuple ActionExekutor::getCommandTuple_st()
{
	PeisTuple* commandTuple = peisk_getTupleIndirectly(my_peis_id, tuple_set_[COMMAND].c_str(), PEISK_KEEP_OLD);
	//ROS_INFO("Fetching Command for action...");
	while(!commandTuple)
	{
		commandTuple = peisk_getTupleIndirectly(my_peis_id, tuple_set_[COMMAND].c_str(), PEISK_KEEP_OLD);
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

PeisTuple ActionExekutor::getStateTuple_st()
{
	PeisTuple* stateTuple = peisk_getTuple(my_peis_id, tuple_set_[STATE].c_str(), PEISK_KEEP_OLD);
	//ROS_INFO("Fetching state.");
	while(!stateTuple)
	{
		stateTuple = peisk_getTuple(my_peis_id, tuple_set_[STATE].c_str(), PEISK_KEEP_OLD);
		sleep(500000);
	}
	return *stateTuple;
}
	
}
