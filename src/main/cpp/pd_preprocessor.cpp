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

#include "pd_preprocessor.h"
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

MODULE_DEF(module_pd_preprocessor, module_pd_preprocessor::pd_preprocessor);

using namespace std;
using namespace robotkernel;
using namespace module_pd_preprocessor;
using namespace string_util;

//! default construction
/*!
 * \param node yaml configuration node
 */
pd_preprocessor::pd_preprocessor(const char* name, const YAML::Node& node) : 
    pd_provider(name), pd_consumer(name),
    runnable(node), module_base("module_pd_preprocessor", name, node) 
{
}

//! destrcution
pd_preprocessor::~pd_preprocessor() {
}
        
// additional module init stuff
void pd_preprocessor::init() {
}

//! handler function called if thread is running
void pd_preprocessor::run() {
}

//! set module state machine to defined state
/*!
  \param state requested state
  \return success or failure
  */
int pd_preprocessor::set_state(module_state_t state) {
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

