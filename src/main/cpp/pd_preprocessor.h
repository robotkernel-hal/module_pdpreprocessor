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

#ifndef __MODULE_POSIX_TIMER_H__
#define __MODULE_POSIX_TIMER_H__

#include "robotkernel/module_intf.h"
#include "robotkernel/module_base.h"
#include "robotkernel/runnable.h"
#include "robotkernel/kernel.h"
#include "robotkernel/trigger_base.h"

namespace module_posix_timer {
#ifdef EMACS
}
#endif

// forward declaration
class posix_timer : 
    public std::enable_shared_from_this<posix_timer>,
    public robotkernel::pd_provider,
    public robotkernel::trigger,
    public robotkernel::runnable, 
    public robotkernel::module_base 
{
    private:
        posix_timer();                               //!< prevent default cons
        posix_timer(const posix_timer&);             //!< prevent copy-construction
        posix_timer& operator=(const posix_timer&);  //!< prevent assignment

    public:
        int signo;                                   //!< signal number
        timer_t timer_id;                            //!< timer id

        enum posix_timer_mode {
            posix_timer_mode_nanosleep,
            posix_timer_mode_timer,
        } mode;

        robotkernel::sp_process_data_t pdin;         //!< named process data
        size_t provider_hash;

        //! default construction
        /*!
         * \param node yaml configuration node
         */
        posix_timer(const char *name, const YAML::Node& node);

        //! destrcution
        ~posix_timer();

        //! additional module init stuff
        void init();

        //! set rate of trigger device
        /*!
         * set the rate of the current trigger
         * overload in derived trigger class
         *
         * \param new_rate new trigger rate to set
         */
        void set_rate(double new_rate);

        //! set module state machine to defined state
        /*!
          \param state requested state
          \return success or failure
          */
        int set_state(module_state_t state);

        //! handler function called if thread is running
        void run();

        //! handler function for nanosleep mode
        void run_nanosleep();

        //! handler function for timer mode
        void run_timer();
};

#ifdef EMACS
{
#endif
};

#endif // __MODULE_POSIX_TIMER_H__

