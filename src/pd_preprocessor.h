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

#ifndef MODULE_PD_PREPROCESSOR__PD_PREPROCESSOR_H
#define MODULE_PD_PREPROCESSOR__PD_PREPROCESSOR_H

#include "robotkernel/module_base.h"
#include "robotkernel/trigger_base.h"
#include "robotkernel/process_data.h"

#include "service_provider_key_value/base.h"
#include "service_provider_key_value/key_value_helper.h"

namespace module_pd_preprocessor {

// forward declarations
class preproc_device;
class pd_preprocessor;

typedef struct {
    robotkernel::sp_trigger_t trigger;
    robotkernel::sp_process_data_t pd;
    size_t length;
    robotkernel::sp_pd_provider_t provider;
    robotkernel::sp_pd_consumer_t consumer;
} pd_t;

class preproc_entry 
{
    public:
        std::string field_name;
        std::string cast_to;
        std::string convert_to;
        std::string alias;
        double scaling;
        double offset;
        int64_t raw_offset;
        bool hide;
        bool just_copy;

        robotkernel::pd_data_types cast_to_dt;
        robotkernel::pd_data_types convert_to_dt;

        size_t export_len;
        off_t export_offset;

        size_t import_len;
        off_t import_offset;
        robotkernel::pd_data_types import_dt;
    public:
        preproc_entry() :
            field_name(""), cast_to(""), convert_to(""), scaling(1.), offset(0.), raw_offset(0), hide(false), just_copy(true)
        { };
        preproc_entry(const YAML::Node& node);
};

class preproc_device :
    public std::enable_shared_from_this<preproc_device>,
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

        void close();

        // trigger function
        virtual void tick() override;
};


// forward declaration
class pd_preprocessor : 
    public std::enable_shared_from_this<pd_preprocessor>,
    public robotkernel::module_base
{
    private:
        pd_preprocessor();                               //!< prevent default cons
        pd_preprocessor(const pd_preprocessor&);             //!< prevent copy-construction
        pd_preprocessor& operator=(const pd_preprocessor&);  //!< prevent assignment

    public:

        std::shared_ptr<service_provider_key_value::slave> kvs;

        robotkernel::sp_process_data_t pdin;         //!< named process data
        robotkernel::sp_pd_provider_t pdin_provider;

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
        virtual void init() override;

        //! State transition from PREOP to INIT
        virtual void set_state_preop_2_init() override; 

        //! State transition from INIT to PREOP
        virtual void set_state_init_2_preop() override;
};

}; // namespace module_pd_preprocessor

#endif // MODULE_PD_PREPROCESSOR__PD_PREPROCESSOR_H

