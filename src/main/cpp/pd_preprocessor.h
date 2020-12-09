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

#ifndef __MODULE_PD_PREPROCESSOR_H__
#define __MODULE_PD_PREPROCESSOR_H__

#include "robotkernel/module_intf.h"
#include "robotkernel/module_base.h"
#include "robotkernel/runnable.h"
#include "robotkernel/kernel.h"
#include "robotkernel/trigger_base.h"

namespace module_pd_preprocessor {
#ifdef EMACS
}
#endif

// forward declarations
class pd_preprocessor;

typedef struct {
    robotkernel::sp_trigger_t trigger;
    robotkernel::sp_process_data_t pd;
    size_t length;
    size_t hash;
} pd_t;

class preproc_entry 
{
    public:
        std::string field_name;
        std::string cast_to;
        std::string convert_to;
        double scaling;
        bool hide;

    public:
        preproc_entry();
        preproc_entry(const YAML::Node& node);
};

class preproc_device :
    public std::enable_shared_from_this<preproc_device>,
    public robotkernel::pd_provider,
    public robotkernel::pd_consumer,
    public robotkernel::trigger_base
{
    public: 
        std::shared_ptr<pd_preprocessor> parent;
        std::string name;

        std::string type;
        std::string pd_name;
        pd_t import_pd;
        pd_t export_pd;
        std::map<std::string, preproc_entry> entries;
    public:
        // construction
        preproc_device(const std::string& name, 
                std::shared_ptr<pd_preprocessor> parent, const YAML::Node& node);
        // destruction
        ~preproc_device();

        void open();

        // trigger function
        void tick();
};


// forward declaration
class pd_preprocessor : 
    public std::enable_shared_from_this<pd_preprocessor>,
    public robotkernel::pd_provider,
    public robotkernel::pd_consumer,
    public robotkernel::runnable, 
    public robotkernel::module_base 
{
    private:
        pd_preprocessor();                               //!< prevent default cons
        pd_preprocessor(const pd_preprocessor&);             //!< prevent copy-construction
        pd_preprocessor& operator=(const pd_preprocessor&);  //!< prevent assignment

    public:

        robotkernel::sp_process_data_t pdin;         //!< named process data
        size_t provider_hash;

        YAML::Node node;
        std::list<std::shared_ptr<preproc_device> > devices;

        //! default construction
        /*!
         * \param node yaml configuration node
         */
        pd_preprocessor(const char *name, const YAML::Node& node);

        //! destrcution
        ~pd_preprocessor();

        //! additional module init stuff
        void init();

        //! set module state machine to defined state
        /*!
          \param state requested state
          \return success or failure
          */
        int set_state(module_state_t state);

        //! handler function called if thread is running
        void run();
};

#ifdef EMACS
{
#endif
};

#endif // __MODULE_PD_PREPROCESSOR_H__

