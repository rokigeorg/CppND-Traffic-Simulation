#include <iostream>
#include <random>
#include "TrafficLight.h"
#include <chrono>

#include "TrafficLight.h"
#include <queue>
#include <future>


/* Implementation of class "MessageQueue" */

template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 

    std::unique_lock<std::mutex> uLock(_mtx);
    _cond.wait(uLock, [this] { return !_queue.empty(); });

    T msg = std::move(_queue.back());
    _queue.pop_back();
    return msg;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.

    std::lock_guard<std::mutex> lck(_mtx);
    _queue.push_back(std::move(msg));
    _cond.notify_one();
}


/* Implementation of class "TrafficLight" */


TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
    _msgqueue = std::make_shared<MessageQueue<TrafficLightPhase>>();

}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.

    while (true)
	{
		/* Sleep at every iteration to reduce CPU usage */
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

		/* Wait until the traffic light is green, received from message queue */
		TrafficLightPhase curr_phase = _msgqueue->receive();
		if (curr_phase == green)
		{
			return;
		}
	}
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class. 

    // launch cycleThroughPhase function in a thread
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
    // and toggles the current phase of the traffic light between red and green and sends an update method 
    // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds. 
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles. 


	/* Init our random generation between 4 and 6 seconds */
	std::random_device rd;
	std::mt19937 eng(rd());
	std::uniform_int_distribution<> distr(4, 6);

	/* Print id of the current thread */
	std::unique_lock<std::mutex> lck(_mutex);
	std::cout << "Traffic_Light #" << _id << "::Cycle_Through_Phases: thread id = " << std::this_thread::get_id() << std::endl;
	lck.unlock();


    /* Initalize variables */
	int cycle_duration = distr(eng); //Duration of a single simulation cycle in seconds, is randomly chosen

    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();

    while(true)
    {
        /* Compute time difference to stop watch */
		long time_since_last_update = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - t1).count();

        // simulate some work
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if(time_since_last_update  >= cycle_duration )
        {
            _currentPhase = _currentPhase == red ? TrafficLightPhase::green : TrafficLightPhase::red;
           
            // create message
            auto msg = _currentPhase;
            auto isSend = std::async(std::launch::async, &MessageQueue<TrafficLightPhase>::send, _msgqueue, std::move(msg));
			isSend.wait();

            //update stop watch
            t1 = std::chrono::high_resolution_clock::now();

        	/* Randomly choose the cycle duration for the next cycle */
			cycle_duration = distr(eng);
        }

    }
}