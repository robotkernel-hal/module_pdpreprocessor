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
        
preproc_entry::preproc_entry(const YAML::Node& node) {
    field_name = get_as<string> (node, "field_name");
    alias      = get_as<string> (node, "alias", "");
    cast_to    = get_as<string> (node, "cast_to", "");
    convert_to = get_as<string> (node, "convert_to", "");
    scaling    = get_as<double> (node, "scaling", 1.);
    offset     = get_as<double> (node, "offset", 0.);
    raw_offset = get_as<int64_t>(node, "raw_offset", 0);
    hide       = get_as<bool>   (node, "hide", false);

    just_copy  = (cast_to == "") && (convert_to == "") && (scaling == 1.);
        
    cast_to_dt = PD_DT_NONE;
    convert_to_dt = PD_DT_NONE;

    export_len = 0;
    export_offset = 0;

    import_len = 0;
    import_offset = 0;
    import_dt = PD_DT_NONE;

    if (convert_to != "") {
        convert_to_dt = process_data::get_data_type(convert_to);
    }
    
    if (cast_to != "") {
        cast_to_dt = process_data::get_data_type(cast_to);
    }
}

// construction
preproc_device::preproc_device(const std::string& name, 
        std::shared_ptr<pd_preprocessor> parent, const YAML::Node& node) :
    pd_provider(name), pd_consumer(name), 
    parent(parent), name(name)
{
    type = get_as<string>(node, "type");
    pd_name = get_as<string>(node, "pd_name");

    for (const auto& entries_node : node["entries"]) {
        preproc_entry entry(entries_node);
        entries[entry.field_name] = entry;
    }
}

// destruction
preproc_device::~preproc_device() {
}

template<>
inline void key_value_eval<double>(double* ptr, std::string repr) {
    *ptr = atof(repr.c_str());
}

template<>
inline std::string key_value_repr<double>(double& value) {
    return string_util::format_string("%f", value);
}

void preproc_device::open() {
    kernel& k = *kernel::get_instance();

    parent->log(info, "%s: trying to get %s\n", name.c_str(), pd_name.c_str());

    import_pd.pd = k.get_process_data(pd_name);
    if (import_pd.pd->clk_device != "") {
        parent->log(info, "%s: trying to get %s\n", name.c_str(), import_pd.pd->clk_device.c_str());
        import_pd.trigger = k.get_trigger(import_pd.pd->clk_device);
    }

    // create new process data description
    YAML::Node pd_desc = YAML::Load(import_pd.pd->process_data_definition);

    YAML::Emitter emitter;
    emitter << YAML::BeginSeq;
    
    export_pd.length = 0;

    off_t import_offset = 0, export_offset = 0;

    // check if all fields exists in PD
    for (auto const& e : entries) {
        bool exists = false;
        for (const auto& pd_desc_entry : pd_desc) {
            for (const auto& kv : pd_desc_entry) {
                string value = kv.second.as<string>();
                if (value.find(e.first) != std::string::npos) {
                    exists = true;
                }
            }
        }
        if (!exists)
            parent->log(warning, "field '%s' not found in PD\n", e.first.c_str());
    }

    for (const auto& pd_desc_entry : pd_desc) {
        emitter << YAML::BeginMap;
        
        for (const auto& kv : pd_desc_entry) {
            string key = kv.first.as<string>();
            string value = kv.second.as<string>();
            bool hide = false;
                
            // calculate import offset
            pd_data_types import_dt = process_data::get_data_type(key);
            ssize_t dt_len = process_data::get_data_type_length(key);
            if (dt_len == -1) {
                parent->log(error, "unknown data_type %s\n", key.c_str());
            }

            if (entries.find(value) != entries.end()) {
                auto& e = entries[value];
                hide = e.hide;

                if (e.convert_to != "") {
                    key = e.convert_to;
                } else if (e.cast_to != "") {
                    key = e.cast_to;
                }
            } else {
                entries[value].field_name = value;
            }

            entries[value].import_len = dt_len;
            entries[value].import_offset = import_offset;
            entries[value].import_dt = import_dt;
            import_offset += dt_len;

            string e_name = value;
            if (entries[value].alias != "") {
                e_name = entries[value].alias;
            }
                
            if (!hide) {
                parent->log(info, "%s: adding %s : %s\n", name.c_str(), key.c_str(), e_name.c_str());
                emitter << YAML::Key << key << YAML::Value << e_name;

                dt_len = process_data::get_data_type_length(key);
                if (dt_len == -1) {
                    parent->log(error, "unknown data_type %s\n", key.c_str());
                }

                entries[value].import_len = dt_len;
                entries[value].export_offset = export_offset;
                export_offset += dt_len;
                export_pd.length += dt_len;
            }
        }

        emitter << YAML::EndMap;
    }

    emitter << YAML::EndSeq;

    if (type == "inputs") {
        export_pd.trigger = make_shared<trigger>(parent->name, name + ".inputs");
        export_pd.pd = make_shared<triple_buffer>(export_pd.length, parent->name,
                name + ".inputs", emitter.c_str(), export_pd.trigger->id());

        import_pd.hash = import_pd.pd->set_consumer(shared_from_this());
        export_pd.hash = export_pd.pd->set_provider(shared_from_this());

        if (import_pd.trigger != nullptr) {
            import_pd.trigger->add_trigger(shared_from_this());
        }

        k.add_device(export_pd.trigger);
        k.add_device(export_pd.pd);
    } else {
        export_pd.trigger = make_shared<trigger>(parent->name, name + ".outputs");
        export_pd.pd = make_shared<triple_buffer>(export_pd.length, parent->name,
                name + ".outputs", emitter.c_str(), export_pd.trigger->id());
        
        import_pd.hash = import_pd.pd->set_provider(shared_from_this());
        export_pd.hash = export_pd.pd->set_consumer(shared_from_this());
        
        export_pd.trigger->add_trigger(shared_from_this());

        k.add_device(export_pd.trigger);
        k.add_device(export_pd.pd);
    }
        /*
        std::string field_name;
        std::string cast_to;
        std::string convert_to;
        std::string alias;
        double scaling;
        double offset;
        int64_t raw_offset;
        bool hide;
        bool just_copy;
        */
}

void preproc_device::close() {
    kernel& k = *kernel::get_instance();
    
    k.remove_device(export_pd.pd);
    k.remove_device(export_pd.trigger);

    if (type == "inputs") {
        if (import_pd.trigger != nullptr) {
            import_pd.trigger->remove_trigger(shared_from_this());
        }

        export_pd.pd->reset_provider(export_pd.hash);
        export_pd.hash = 0;

        import_pd.pd->reset_consumer(import_pd.hash);
        import_pd.hash = 0;
    } else {
        export_pd.trigger->remove_trigger(shared_from_this());

        export_pd.pd->reset_consumer(export_pd.hash);
        export_pd.hash = 0;

        import_pd.pd->reset_provider(import_pd.hash);
        import_pd.hash = 0;
    }

    export_pd.pd = nullptr;
    export_pd.trigger = nullptr;
}

template <typename in_dt, typename out_dt> 
static inline void convert_to(double scaling, const double& offset, const int64_t& raw_offset,
        const uint8_t *in_buf, const uint8_t *out_buf) {
    out_dt out = scaling * (*((in_dt *)in_buf) + raw_offset) + offset;
    *((out_dt *)out_buf) = out;
}

template <typename convert_dt>
static inline void convert_to_switch(pd_data_types import_dt, double scaling, const double& offset, const int64_t& raw_offset,
        const uint8_t *in_buf, const uint8_t *out_buf) {
    switch (import_dt) {
        case PD_DT_FLOAT:
            convert_to<float, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_DOUBLE:
            convert_to<double, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_UINT8:
            convert_to<uint8_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_UINT16:
            convert_to<uint16_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_UINT32:
            convert_to<uint32_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_INT8:
            convert_to<int8_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_INT16:
            convert_to<int16_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_INT32:
            convert_to<int32_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        default: break;
    }
}

template <typename in_dt, typename out_dt> 
static inline void convert_to_out(double scaling, const double& offset, const int64_t& raw_offset,
        const uint8_t *in_buf, const uint8_t *out_buf) {
    out_dt out = scaling * (*((in_dt *)in_buf) + offset) + raw_offset;
    *((out_dt *)out_buf) = out;
}

template <typename convert_dt>
static inline void convert_to_switch_out(pd_data_types import_dt, double scaling, const double& offset, const int64_t& raw_offset,
        const uint8_t *in_buf, const uint8_t *out_buf) {
    switch (import_dt) {
        case PD_DT_FLOAT:
            convert_to_out<float, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_DOUBLE:
            convert_to_out<double, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_UINT8:
            convert_to_out<uint8_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_UINT16:
            convert_to_out<uint16_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_UINT32:
            convert_to_out<uint32_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_INT8:
            convert_to_out<int8_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_INT16:
            convert_to_out<int16_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        case PD_DT_INT32:
            convert_to_out<int32_t, convert_dt>(scaling, offset, raw_offset, in_buf, out_buf);
            break;
        default: break;
    }
}

void preproc_device::tick() {
    if (type == "inputs") {
        const auto& import_buf = import_pd.pd->pop(import_pd.hash);
        const auto& export_buf = export_pd.pd->next(export_pd.hash);

        for (const auto& kv : entries) {
            auto& e = kv.second;
            uint8_t *import_val = &import_buf[e.import_offset];
            uint8_t *export_val = &export_buf[e.export_offset];

            if (e.just_copy) {
                memcpy(export_val, import_val, e.import_len);
            } else {
                pd_data_types import_dt = e.import_dt, export_dt;
                if (e.cast_to != "") {
                    import_dt = e.cast_to_dt;
                }

                if (e.convert_to != "") {
                    export_dt = e.convert_to_dt;
                } else {
                    export_dt = import_dt;
                }

                switch (export_dt) {
                    case PD_DT_UNKNOWN:
                    case PD_DT_NONE:
                        break;
                    case PD_DT_FLOAT:
                        convert_to_switch<float>(import_dt, e.scaling, e.offset, e.raw_offset, import_val, export_val);
                        break;
                    case PD_DT_DOUBLE:
                        convert_to_switch<double>(import_dt, e.scaling, e.offset, e.raw_offset, import_val, export_val);
                        break;
                    case PD_DT_UINT8:
                        convert_to_switch<uint8_t>(import_dt, e.scaling, e.offset, e.raw_offset, import_val, export_val);
                        break;
                    case PD_DT_UINT16:
                        convert_to_switch<uint16_t>(import_dt, e.scaling, e.offset, e.raw_offset, import_val, export_val);
                        break;
                    case PD_DT_UINT32:
                        convert_to_switch<uint32_t>(import_dt, e.scaling, e.offset, e.raw_offset, import_val, export_val);
                        break;
                    case PD_DT_INT8:
                        convert_to_switch<int8_t>(import_dt, e.scaling, e.offset, e.raw_offset, import_val, export_val);
                        break;
                    case PD_DT_INT16:
                        convert_to_switch<int16_t>(import_dt, e.scaling, e.offset, e.raw_offset, import_val, export_val);
                        break;
                    case PD_DT_INT32:
                        convert_to_switch<int32_t>(import_dt, e.scaling, e.offset, e.raw_offset, import_val, export_val);
                        break;
                }
            }
        }

        export_pd.pd->push(export_pd.hash);

        if (export_pd.trigger != nullptr) {
            export_pd.trigger->trigger_modules();
        }
    } else {
        const auto& import_buf = import_pd.pd->next(import_pd.hash);
        const auto& export_buf = export_pd.pd->pop(export_pd.hash);

        for (const auto& kv : entries) {
            auto& e = kv.second;
            uint8_t *import_val = &import_buf[e.import_offset];
            uint8_t *export_val = &export_buf[e.export_offset];

            if (e.just_copy) {
                memcpy(import_val, export_val, e.import_len);
            } else {
                pd_data_types import_dt = e.import_dt;
                if (e.cast_to != "") {
                    import_dt = e.cast_to_dt;
                }

                if (e.convert_to != "") {
                    switch (import_dt) {
                        case PD_DT_UNKNOWN:
                        case PD_DT_NONE:
                            break;
                        case PD_DT_FLOAT:
                            convert_to_switch_out<float>(e.convert_to_dt, e.scaling, e.offset, e.raw_offset, export_val, import_val);
                            break;
                        case PD_DT_DOUBLE:
                            convert_to_switch_out<double>(e.convert_to_dt, e.scaling, e.offset, e.raw_offset, export_val, import_val);
                            break;
                        case PD_DT_UINT8:
                            convert_to_switch_out<uint8_t>(e.convert_to_dt, e.scaling, e.offset, e.raw_offset, export_val, import_val);
                            break;
                        case PD_DT_UINT16:
                            convert_to_switch_out<uint16_t>(e.convert_to_dt, e.scaling, e.offset, e.raw_offset, export_val, import_val);
                            break;
                        case PD_DT_UINT32:
                            convert_to_switch_out<uint32_t>(e.convert_to_dt, e.scaling, e.offset, e.raw_offset, export_val, import_val);
                            break;
                        case PD_DT_INT8:
                            convert_to_switch_out<int8_t>(e.convert_to_dt, e.scaling, e.offset, e.raw_offset, export_val, import_val);
                            break;
                        case PD_DT_INT16:
                            convert_to_switch_out<int16_t>(e.convert_to_dt, e.scaling, e.offset, e.raw_offset, export_val, import_val);
                            break;
                        case PD_DT_INT32:
                            convert_to_switch_out<int32_t>(e.convert_to_dt, e.scaling, e.offset, e.raw_offset, export_val, import_val);
                            break;
                    }
                }
            }
        }

        import_pd.pd->push(import_pd.hash);
    }
}

 //! default construction
/*!
 * \param node yaml configuration node
 */
pd_preprocessor::pd_preprocessor(const char* name, const YAML::Node& node) : 
    pd_provider(name), pd_consumer(name),
    runnable(node), module_base("module_pd_preprocessor", name, node)
{
    this->node = YAML::Clone(node);
}

//! destrcution
pd_preprocessor::~pd_preprocessor() {
}
        
// additional module init stuff
void pd_preprocessor::init() {
    for (const auto& kv : node["devices"]) {
        auto sdev = make_shared<preproc_device>(kv.first.as<string>(),
                shared_from_this(), kv.second);
        devices.push_back(sdev);
    }

    std::list<YAML::Node> instances_list;
    robotkernel::parse_templates(node, instances_list);
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
            k.remove_device(kvs);
            kvs = nullptr;

            // ====> deinit devices
            for (const auto& sdev : devices) {
                sdev->close();
            }

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
            kvs = make_shared<key_value_slave>(name, "parameters");
            k.add_device(kvs);

            for (const auto& sdev : devices) {
                sdev->open();
    
                for (auto& kv : sdev->entries) {
                    auto& e    = kv.second;

                    {
                        auto *kvk = new key_value_key<double>(kvs.get(),
                                format_string("%s.%s.scaling", sdev->name.c_str(), e.field_name.c_str()), &e.scaling, false);
                        kvs->_add_key(kvk); 
                    }

                    {
                        auto *kvk = new key_value_key<double>(kvs.get(),
                                format_string("%s.%s.offset", sdev->name.c_str(), e.field_name.c_str()), &e.offset, false);
                        kvs->_add_key(kvk); 
                    }

                    {
                        auto *kvk = new key_value_key<int64_t>(kvs.get(),
                                format_string("%s.%s.raw_offset", sdev->name.c_str(), e.field_name.c_str()), &e.raw_offset, false);
                        kvs->_add_key(kvk); 
                    }
                }
            }

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

