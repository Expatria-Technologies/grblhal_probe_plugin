/*

  probe_plugin.c

  Part of grblHAL

  grblHAL is
  Copyright (c) 2022-2023 Terje Io

  Probe Protection code is
  Copyright (c) 2023 Expatria Technologies

  Public domain.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

  M401   - Set probe connected.
  M402   - Clear probe Connected.

  NOTES: The symbol TOOLSETTER_RADIUS (defined in grbl/config.h, default 5.0mm) is the tolerance for checking "@ G59.3".
         When $341 tool change mode 1 or 2 is active it is possible to jog to/from the G59.3 position.
         Automatic hard-limit switching when probing at the G59.3 position requires the machine to be homed (X and Y).

  Tip: Set default mode at startup by adding M401 to a startup script ($N0 or $N1)

*/

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "grbl/nvs_buffer.h"

#include "probe_plugin.h"

#define RELAY_DEBOUNCE 50 // ms - increase if relay is slow and/or bouncy

#define PROBE_PLUGIN_PORT_SETTING Setting_UserDefined_0
#define PROBE_PLUGIN_FIXTURE_INVERT_LIMIT_SETTING Setting_UserDefined_1



//add function pointers for tool number and pulse start


static uint8_t n_ports;
static char max_port[4];

typedef struct {
    uint8_t port;
    uint8_t invert_limits;
} probe_protect_settings_t;


static uint8_t probe_connect_port;
static bool probe_connected = false;
static driver_reset_ptr driver_reset;
static user_mcode_ptrs_t user_mcode;

static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static probe_connected_toggle_ptr probe_connected_toggle;
static probe_protect_settings_t probe_protect_settings;
static on_probe_start_ptr on_probe_start;
static on_probe_completed_ptr on_probe_completed;
static on_probe_fixture_ptr on_probe_fixture;
static on_spindle_select_ptr on_spindle_select;
static spindle_set_state_ptr on_spindle_set_state = NULL;

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)401 || mcode == (user_mcode_t)402
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t mcode_validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    switch((uint16_t)gc_block->user_mcode) {

        case 401:
            break;

        case 402:
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void probe_start (void){

    if(on_probe_start)
        on_probe_start();
}

static void probe_completed (void){


    if(on_probe_completed)
        on_probe_completed();
}

// When called from "normal" probing tool is always NULL, when called from within
// a tool change sequence (M6) then tool is a pointer to the selected tool.
bool probe_fixture (tool_data_t *tool, bool at_g59_3, bool on)
{
    //set polarity before probing the fixture

    //set hard limits before probing the fixture.

    //hal.limits.enable(settings.limits.flags.hard_enabled, false); // Change immediately. NOTE: Nice to have but could be problematic later.

    hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.

    if(on_probe_fixture)
        on_probe_fixture();

    return true;
}

static void probe_safety_check(void){
    //use on spindle select to set the onSpindleSetState pointer so that things are event driven.
    //hal.spindle_data.get(SpindleData_RPM)->rpm  
}

static void on_probe_motion(void){
    //turn off halt generation on a probing motion.
}

static void on_probe_connected_toggle(void){
    //on each pulse start, check probe pin (maybe look for something in the block planner instead?)

    if(probe_connected_toggle)
        probe_connected_toggle();

}

static void onSpindleSetState (spindle_state_t state, float rpm)
{
    uint32_t idx = FANS_ENABLE;
    do {
        if(bit_true(fan_setting.spindle_link, bit(--idx))) {

            if(!state.on && bit_isfalse(fans_linked, bit(idx)))
                continue;

            if(state.on && !fan_get_state(idx))
                bit_true(fans_linked, bit(idx));

            if(idx == 0 && !state.on && fan_setting.fan0_off_delay > 0.0f) {
                fan_off = hal.get_elapsed_ticks();
                fan_off_delay = (uint32_t)(fan_setting.fan0_off_delay * 60.0f) * 1000;
            } else
                fan_set_state(idx, state.on);
        }
    } while(idx);

    on_spindle_set_state(state, rpm);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{
    on_spindle_set_state = spindle->set_state;
    spindle->set_state = onSpindleSetState;

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    if (state != STATE_CHECK_MODE)
      switch((uint16_t)gc_block->user_mcode) {

        case 401:
            if(!probe_connected){
                probe_connected = true;
                //enqueue probe connected symbol.
                grbl.enqueue_realtime_command(CMD_PROBE_CONNECTED_TOGGLE);
                hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
            }else
                report_message("Probe connected signal already asserted!", Message_Warning);
            break;

        case 402:
            if(probe_connected){
                probe_connected = false;
                //enqueue probe disconnected symbol.
                grbl.enqueue_realtime_command(CMD_PROBE_CONNECTED_TOGGLE);
                hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
            }else
                report_message("Probe connected signal not asserted!", Message_Warning);
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void probe_reset (void)
{
    driver_reset();
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Probe Protection v0.01]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Probe protect plugin failed to initialize!", Message_Warning);
}

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.

static const setting_group_detail_t user_groups [] = {
    { Group_Root, Group_Probing, "Probe Protection"}
};

static const setting_detail_t user_settings[] = {
    { PROBE_PLUGIN_PORT_SETTING, Group_Probing, "Relay aux port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &probe_protect_settings.port, NULL, NULL },
    { PROBE_PLUGIN_FIXTURE_INVERT_LIMIT_SETTING, Group_Probing, "Tool Probe Invert", NULL, Format_Bitfield, "Enable,Hard Limits", NULL, NULL, Setting_NonCore, &probe_protect_settings.invert_limits, NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t probe_protect_settings_descr[] = {
    { PROBE_PLUGIN_PORT_SETTING, "Aux input port number to use for probe connected control.\\n\\n"
                            "NOTE: A hard reset of the controller is required after changing this setting."
    },
    { PROBE_PLUGIN_FIXTURE_INVERT_LIMIT_SETTING, "Inversion setting for Probe signal during tool measurement.\\n\\n"
                            "NOTE: A hard reset of the controller is required after changing this setting."
    },   
};

#endif

// Write settings to non volatile storage (NVS).
static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&probe_protect_settings, sizeof(probe_protect_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
// Default is highest numbered free port.
static void plugin_settings_restore (void)
{
    probe_protect_settings.port = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&probe_protect_settings, sizeof(probe_protect_settings_t), true);
}

static void warning_no_port (uint_fast16_t state)
{
    report_message("Probe plugin: configured port number is not available", Message_Warning);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&probe_protect_settings, nvs_address, sizeof(probe_protect_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();

    // Sanity check
    if(probe_protect_settings.port >= n_ports)
        probe_protect_settings.port = n_ports - 1;

    probe_connect_port = probe_protect_settings.port;

    if(ioport_claim(Port_Digital, Port_Input, &probe_connect_port, "Probe Connected")) {

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = mcode_check;
        hal.user_mcode.validate = mcode_validate;
        hal.user_mcode.execute = mcode_execute;

    } else
        protocol_enqueue_rt_command(warning_no_port);
}

// Settings descriptor used by the core when interacting with this plugin.
static setting_details_t setting_details = {
    .groups = user_groups,
    .n_groups = sizeof(user_groups) / sizeof(setting_group_detail_t),
    .settings = user_settings,
    .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = probe_protect_settings_descr,
    .n_descriptions = sizeof(probe_protect_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = plugin_settings_save,
    .load = plugin_settings_load,
    .restore = plugin_settings_restore
};

void probe_protect_init (void)
{
    bool ok = false;

    //subscribe to tool function pointer and probe connect toggle.
    probe_connected_toggle = hal.probe.connected_toggle;
    hal.probe.connected_toggle = on_probe_connected_toggle;

    //subscribe to probe fixture event.
    on_probe_fixture = grbl.on_probe_fixture;
    grbl.on_probe_fixture = probe_fixture;

    on_probe_completed = grbl.on_probe_completed;
    grbl.on_probe_completed = probe_completed;

    on_probe_start = grbl.on_probe_start;
    grbl.on_probe_start = probe_start;

    on_spindle_select = grbl.on_spindle_select;
    grbl.on_spindle_select = onSpindleSelect;

    driver_reset = hal.driver_reset;
    hal.driver_reset = probe_reset;

    if(!ioport_can_claim_explicit()) {

        // Driver does not support explicit pin claiming, claim the highest numbered port instead.

        if((ok = hal.port.num_digital_out > 0)) {

            probe_connect_port = --hal.port.num_digital_in;        // "Claim" the port, M62-M65 cannot be used
    //        relay_port = hal.port.num_digital_out - 1;    // Do not "claim" the port, M62-M65 can be used

            if(hal.port.set_pin_description)
                hal.port.set_pin_description(Port_Digital, Port_Input, probe_connect_port, "Probe relay");

            memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

            hal.user_mcode.check = mcode_check;
            hal.user_mcode.validate = mcode_validate;
            hal.user_mcode.execute = mcode_execute;

            driver_reset = hal.driver_reset;
            hal.driver_reset = probe_reset;

            on_report_options = grbl.on_report_options;
            grbl.on_report_options = report_options;

        }

    } else if((ok = (n_ports = ioports_available(Port_Digital, Port_Input)) > 0 && (nvs_address = nvs_alloc(sizeof(probe_protect_settings_t))))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        settings_register(&setting_details);

        // Used for setting value validation
        strcpy(max_port, uitoa(n_ports - 1));
    }

    if(!ok)
        protocol_enqueue_rt_command(warning_msg);
}

