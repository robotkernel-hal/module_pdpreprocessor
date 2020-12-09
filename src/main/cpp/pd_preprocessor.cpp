//! robotkernel module posix timer
/*!
 * author: Robert Burger <robert.burger@dlr.de>
 */

/*
 * This file is part of robotkernel.
 *
 * robotkernel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * robotkernel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with robotkernel.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "posix_timer.h"
#include "robotkernel/helpers.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <semaphore.h>
#include <signal.h>
#include <pthread.h>

#include <iostream>
#include <chrono>

#include "yaml-cpp/yaml.h"
#include <string_util/string_util.h>

MODULE_DEF(module_posix_timer, module_posix_timer::posix_timer);

using namespace std;
using namespace robotkernel;
using namespace module_posix_timer;
using namespace string_util;

//! default construction
/*!
 * \param node yaml configuration node
 */
posix_timer::posix_timer(const char* name, const YAML::Node& node) : 
    pd_provider(name),
    trigger(name, "inputs", 1./get_as<double>(node, "interval")),
    runnable(node), module_base("module_posix_timer", name, node) 
{
    signo       = get_as<int>(node, "signo", SIGRTMIN);
    timer_id    = NULL;
    mode        = posix_timer_mode_timer;
    thread_name = name;

    if (node["mode"]) {
        if (node["mode"].as<string>() == string("nanosleep"))
            mode = posix_timer_mode_nanosleep;
        else if (node["mode"].as<string>() == string("timer"))
            mode = posix_timer_mode_timer;
    } else 
        log(info, "mode not specified, assuming timer mode!\n");
};

//! destrcution
posix_timer::~posix_timer() {
}
        
// additional module init stuff
void posix_timer::init() {
}

//! set rate of trigger device
/*!
 * set the rate of the current trigger
 * overload in derived trigger class
 *
 * \param new_rate new trigger rate to set
 */
void posix_timer::set_rate(double new_rate) {
#ifndef TIMER_SET_RATE_ENABLED
    if (mode == posix_timer_mode_timer) {
        throw str_exception("setting rate not supported in timer mode!\n");
    }
#endif

    rate = new_rate;
}

//! handler function called if thread is running
void posix_timer::run() {
    if (mode == posix_timer_mode_nanosleep)
        return run_nanosleep();

    return run_timer();
}

//! handler function for nanosleep mode
void posix_timer::run_nanosleep() {
    log(info, "nanosleep handler running with pid %d\n", getpid());
    auto now = std::chrono::high_resolution_clock::now();
    double interval = 0.;

    while (running()) {
        now += std::chrono::nanoseconds((long)(1000000000. / get_rate()));
        interval = 1. / get_rate();
        pdin->write(provider_hash, 0, (uint8_t *)&interval, sizeof(interval));

        do {
            std::this_thread::sleep_until(now);
        } while (now > std::chrono::high_resolution_clock::now());

        trigger_modules();
    }

    log(info, "nanosleep handler stopped\n");
}

//! handler function for timer mode
void posix_timer::run_timer() {
    log(info, "timer handler running with pid %d\n", getpid());
    
    sigset_t set;
    if (sigemptyset (&set) == -1)
        log(error, "sigemptyset %s\n", strerror(errno));

    if (sigaddset (&set, signo) == -1)
        log(error, "sigaddset %s\n", strerror(errno));

    /* set up timer to send out signal */
    struct sigevent se;
    memset(&se, 0, sizeof(se));
    se.sigev_notify = SIGEV_SIGNAL;
    se.sigev_signo = signo;

    if (timer_create(CLOCK_REALTIME, &se, &timer_id) == -1)
        log(error, "ERROR timer_create: %s\n", strerror(errno));

    double interval = 1. / get_rate();
    double old_interval = interval;
    pdin->write(provider_hash, 0, (uint8_t *)&interval, sizeof(interval));

    struct itimerspec value, value_old; 
    value.it_value.tv_sec = (int)(interval);
    value.it_value.tv_nsec = (interval-value.it_value.tv_sec)*1E9;
    value.it_interval.tv_sec = value.it_value.tv_sec;
    value.it_interval.tv_nsec = value.it_value.tv_nsec;

    if (timer_settime(timer_id, 0, &value, &value_old) == -1)
        log(error, "timer_settime %s\n", strerror(errno));

    while (running()) {
        struct timespec ts = { 1, 0 };
        siginfo_t si;

        int ret = sigtimedwait(&set, &si, &ts);

        if (ret == -1) {
            if (errno == EAGAIN)
                log(info, "sigtimedwait timed out\n");
            if (errno == EINVAL)
                log(info, "sigtimedwait einval\n");
            continue;
        }

// disabled for now
#ifdef TIMER_SET_RATE_ENABLED
        interval = 1. / get_rate();

        if (old_interval != interval) {
            // reload timer with new value
            value.it_value.tv_sec = (int)(interval);
            value.it_value.tv_nsec = (interval-value.it_value.tv_sec)*1E9;
            value.it_interval.tv_sec = value.it_value.tv_sec;
            value.it_interval.tv_nsec = value.it_value.tv_nsec;

            if (timer_settime(timer_id, 0, &value, &value_old) == -1)
                log(error, "timer_settime %s\n", strerror(errno));

            old_interval = interval;
        }
#endif

        trigger_modules();
    }

    if (timer_id) {
        struct itimerspec value; 
        value.it_value.tv_sec = 0;
        value.it_value.tv_nsec = 0;
        value.it_interval.tv_sec = 0;
        value.it_interval.tv_nsec = 0;

        if (timer_settime(timer_id, 0, &value, NULL) == -1)
            log(error, "ERROR timer_settime: %s\n",
                    strerror(errno));

        timer_delete(timer_id);
    }

    log(info, "timer handler stopped\n");
}

//! set module state machine to defined state
/*!
  \param state requested state
  \return success or failure
  */
int posix_timer::set_state(module_state_t state) {
    kernel& k = *kernel::get_instance();

    // get transition
    uint32_t transition = GEN_STATE(this->state, state);

    switch (transition) {
        case op_2_safeop:
        case op_2_preop:
        case op_2_init:
        case op_2_boot:
            // ====> stop sending commands
            if (state == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements
            stop();

            if (state == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
            
            // register devices (trigger, process_data)
            k.remove_device(shared_from_this());
            k.remove_device(pdin);
    
            pdin->reset_provider(provider_hash);
            pdin = nullptr;
            provider_hash = 0;
        case init_2_init:
            // ====> re-/open ethercat device
            if (state == module_state_init)
                break;
        case init_2_boot:
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open ethercat device
            if (state == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop: {
            // ====> initial devices            
            string pdin_desc = "- double: interval\n";
            pdin = make_shared<robotkernel::triple_buffer>(
                    sizeof(double), name, string("inputs"), pdin_desc, trigger::id());

            provider_hash = pdin->set_provider(shared_from_this());
            
            // register devices (trigger, process_data)
            k.add_device(shared_from_this());
            k.add_device(pdin);

            if (state == module_state_preop)
                break;
        }
        case preop_2_op:
        case preop_2_safeop:
            // ====> start receiving measurements
            start();

            if (state == module_state_safeop)
                break;
        case safeop_2_op:
            // ====> start sending commands           
            break;
        case op_2_op:
        case safeop_2_safeop:
        case preop_2_preop:
            // ====> do nothing
            break;

        default:
            break;
    }

    return (this->state = state);
}

