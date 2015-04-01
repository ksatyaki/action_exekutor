/**
 * @file   action_exekutor.h
 * @author Chittaranjan S Srinivas 
 * 
 * @brief  This file declares the ActionExekutor Base class.
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

#ifndef ACTION_EXEKUTOR_H
#define ACTION_EXEKUTOR_H

#include <string>
#include <vector>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <string.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

extern "C" {
#include <peiskernel/peiskernel.h>
#include <peiskernel/peiskernel_mt.h>
}

namespace exekutor
{

/**
 * Forward declaration for typedefs.
 */
class ActionExekutor;

/**
 * Why not use some convenience typedefs?
 */
typedef std::vector<ActionExekutor*> ActionExekutorPtrVector;
typedef std::vector<std::string> StringVector;

/**
 * An enumeration for convenience in accessing meta-tuple related structures.
 */
enum TupleType{COMMAND = 0, STATE, PARAMS, RESULT, PROGRESS};

enum StateValue{COMPLETED = 0, RUNNING, FAILED, IDLE};

/**
 * The macro that defines when an action should time-out.
 */
#define ACTION_TIMEOUT 10

/** A Base class to keep track of all the declared actions and
 * to run a Listener that continuously listens on tuples and
 * spawns the appropriate actionThread() whilst waiting for
 * other tuples whose actions don't affect the actions happening
 */
class ActionExekutor /* ABSTRACT CLASS */
{
 private:
	/**
	 * Reset Meta-tuples that were linked previously.
	 * NOTE: In the finalized set-up the commanding program should reset the link.
	 * This is hence not used at all.
	 */
	void resetMetaTuples();

 protected:
	/**
	 * A Nodehandle.
	 * This ensures that ROS stays alive as long as the last exekutor is alive.
	 */
	ros::NodeHandle nh_;
	
	/**
	 * Id of the thread where this exekutor is running.
	 */
	 pthread_t thread_id_;

	/**
	 * This member stores the name of the robot. Example: Doro1, Oro1, Coro1, Turtlebot342.
	 */
	std::string robot_name_;

	/**
	 * This member stores the name of the action. Example: MoveTo, FindBlob or FindHuman.
	 */
	std::string action_name_;

	/**
	 * Convenience strings to store the Command, State and Parameters meta-keys.
	 */
	std::string tuple_set_[5];

	/**
	 * This static member has a list of action names of all created objects of supertype ActionExekutor.
	 */
	static StringVector action_str_list;

	/**
	 * Peis ID of the process running the exekutor.
	 */
	static int my_peis_id;

	/**
	 * This static member has a list of all the created objects of supertype ActionExekutor.
	 */
	static ActionExekutorPtrVector action_ptr_list;

	/**
	 * Add the action to the action_ptr_list.
	 */
	static inline void addAction(ActionExekutor* _this) { action_ptr_list.push_back(_this); 	action_str_list.push_back(_this->action_name_); }
	    																		/* TODO Do this step if the action is not already on the list */
	/**
	 * The function that makes a call to the actionThread() after doing common tasks.
	 * This takes care of setting the state to RUNNING.
	 */
	void startAction(int i = ACTION_TIMEOUT);

	/**
	 * This function initiates all the peis meta-tuples necessary to use the action.
	 */
	void initiateMetaTuples();

	/**
	 * The function that has to be implemented in all classes that derive from ActionExekutor.
	 *
	 * This function is called from startAction().
	 */
	virtual void actionThread() = 0;
	
	/**
	 * The thread function that actually does the waiting.
	 * One such thread exists for each exekutor running.
	 */
	static void* waitThread(void *_this_);

	/**
	 * Convenience function to get the parameter tuple.
	 */
	PeisTuple getParamTuple();

	/**
	 * Convenience function to get the state tuple.
	 */
	PeisTuple getStateTuple();

	/**
	 * Convenience function to get the command tuple.
	 */
	PeisTuple getCommandTuple();

	/**
	 * Set the STATE tuples to the desired values.
	 */
	void inline setState(StateValue value)
	{
		if(value == COMPLETED)
			peiskmt_setStringTuple(tuple_set_[STATE].c_str(), "COMPLETED");
		else if(value == RUNNING)
			peiskmt_setStringTuple(tuple_set_[STATE].c_str(), "RUNNING");
		else if(value == FAILED)
			peiskmt_setStringTuple(tuple_set_[STATE].c_str(), "FAILED");
		else if(value == IDLE)
			peiskmt_setStringTuple(tuple_set_[STATE].c_str(), "IDLE");
	}

	/**
	 * Set the result tuple for actions like acquire.
	 */
	void inline setResult(std::string result_value)
	{
		peiskmt_setStringTuple(tuple_set_[RESULT].c_str(), result_value.c_str());
	}

	/**
	 * Set the progress tuple.
	 */
	void inline setProgress(std::string progress)
	{
		peiskmt_setStringTuple(tuple_set_[PROGRESS].c_str(), progress.c_str());
	}


 public:

	/**
	 * A transform listener pointer. In practive we create one static Listener.
	 * Actually the original tf_listener_base_ can be used everywhere in code.
	 * To avoid too much rewriting, I have left this as such.
	 */
	boost::shared_ptr<tf::TransformListener> tf_listener_;

	/**
	 * A Listener for all exekutors. All the tf_listener_ pointers point to this only.
	 */
	static boost::shared_ptr <tf::TransformListener> tf_listener_base_;

	/**
	 * Simple constructor that uses the string to initiate all meta tuples related to the action.
	 */
	ActionExekutor(std::string robot_name, std::string a_name, bool listen_on_tf = true);

	/**
	 * Destructor.
	 */
    virtual ~ActionExekutor();

    /**
     * We haven't found the use of this function till now!
     */
    std::string getActionName() {return action_name_;}
    
    /**
     * Debugging function.
     */
	static void printAll();
	
    /**
     * Converts an input string that consists of multiple double values separated by commas or spaces into a vector of doubles.
     * This function converts the c-string given as argument into double values.
     * The double values in the actual string should be separated by commas or spaces.
     */
	std::vector<double> extractParams(const char p[]);

	/**
	 * Converts an input string that consists of multiple strings separated by commas or spaces into a vector of strings.
	 * This function converts the c-string given as argument into a vector of strings.
	 * The strings in the input string should be separated by commas or spaces.
	 */
    std::vector<std::string> extractParamStrings(const char p[]);

    /**
     * When this function is called, the program starts waiting for links to become available
     * so that it can execute actions. This is what should be used when creating an executable.
     */
    static void waitForLink();
    
    /**
     * This call is specific to the exekutor.
     * waitForLink() calls this for every exekutor.
     */
    void waitForMyLink();

    /**
     * Cancel waiting on a particular exekutor.
     * Useful to cancel out conflicting exekutor instances.
     */
    void cancelWaiting();

};

}

#endif // ACTION_EXEKUTOR_H

// Local Variables:
// mode: c++
// End:
