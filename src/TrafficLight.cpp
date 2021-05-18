#include "TrafficLight.h"
#include <chrono>
#include <iostream>
#include <random>

#include <cstdlib>
#include <ctime>
#include <iostream>

#include "TrafficLight.h"
#include <future>
#include <queue>

/* Implementation of class "MessageQueue" */

template <typename T> T MessageQueue<T>::receive() {
  // FP.5a : The method receive should use std::unique_lock<std::mutex> and
  // _condition.wait() to wait for and receive new messages and pull them from
  // the queue using move semantics. The received object should then be returned
  // by the receive function.
  std::unique_lock<std::mutex> uniLock(_mtx);
  _cond.wait(uniLock, [this] { return !_queue.empty(); });

  T msg = std::move(_queue.back());
  _queue.pop_back();
  return msg;
}

template <typename T> void MessageQueue<T>::send(T &&msg) {
  // FP.4a : The method send should use the mechanisms
  // std::lock_guard<std::mutex> as well as _condition.notify_one() to add a new
  // message to the queue and afterwards send a notification.

  std::lock_guard<std::mutex> lck(_mtx);
  _queue.clear();
  _queue.emplace_back(msg);
  _cond.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight() {
  _currentPhase = TrafficLightPhase::red;
  _msgqueue = std::make_shared<MessageQueue<TrafficLightPhase>>();
}

TrafficLight::~TrafficLight() {}

void TrafficLight::waitForGreen() {
  // FP.5b : add the implementation of the method waitForGreen, in which an
  // infinite while-loop runs and repeatedly calls the receive function on the
  // message queue. Once it receives TrafficLightPhase::green, the method
  // returns.

  while (true) {

    TrafficLightPhase curr_phase = _msgqueue->receive();

    std::unique_lock<std::mutex> lck(_mutex);
    std::cout << "Traffic_Light Numb: " << _id
              << " waitForGreen Phaes received = " << curr_phase << std::endl;
    lck.unlock();

    if (TrafficLightPhase::green == curr_phase) {
      return;
    }
  }
}

TrafficLightPhase TrafficLight::getCurrentPhase() { return _currentPhase; }

void TrafficLight::simulate() {
  // FP.2b : Finally, the private method „cycleThroughPhases“ should be started
  // in a thread when the public method „simulate“ is called. To do this, use
  // the thread queue in the base class.

  // launch cycleThroughPhase function in a thread
  threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

static int randNum(int min, int max) { return rand() % max + min; }

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases() {
  // FP.2a : Implement the function with an infinite loop that measures the time
  // between two loop cycles and toggles the current phase of the traffic light
  // between red and green and sends an update method to the message queue using
  // move semantics. The cycle duration should be a random value between 4 and 6
  // seconds. Also, the while-loop should use std::this_thread::sleep_for to
  // wait 1ms between two cycles.

  /* Print id of the current thread */
  std::unique_lock<std::mutex> lck(_mutex);
  std::cout << "Traffic_Light Numb: " << _id
            << "in cycleThroughPhases with thread id = "
            << std::this_thread::get_id() << std::endl;
  lck.unlock();

  long duration = randNum(4, 6); // seconds, is randomly chosen
  std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point t2;

  while (true) {
    t2 = std::chrono::system_clock::now();

    // calc the time which has past
    long time_passed =
        std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();

    if (time_passed >= duration) {
      _currentPhase = _currentPhase == red ? TrafficLightPhase::green
                                           : TrafficLightPhase::red;

      _msgqueue->send(std::move(_currentPhase));

      // reset the initizial timestamp and duration
      duration = randNum(4, 6);
      t1 = std::chrono::system_clock::now();
    }

    // wait for 1ms
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}