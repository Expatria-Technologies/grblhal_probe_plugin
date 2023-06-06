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

#include "probe_plugin.h"

#define RELAY_DEBOUNCE 50 // ms - increase if relay is slow and/or bouncy

#define PROBE_PLUGIN_PORT_SETTING Setting_UserDefined_7
#define PROBE_PLUGIN_FIXTURE_INVERT_LIMIT_SETTING Setting_UserDefined_8



//add function pointers for tool number and pulse start


static uint8_t n_ports;
static char max_port[4];

typedef union {
    uint8_t value;
    struct {
        uint8_t
        invert      :1,
        hardlimits  :1,
        ext_pin     :1,
        ext_pin_inv :1,
        reserved    :4;
    };
} probe_protect_flags_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        toggle      :1,
        mcode       :1,
        ext_pin     :1,
        t99         :1,
        reserved    :4;
    };
} probe_connected_flags_t;

typedef struct {
    uint8_t port;
    probe_protect_flags_t flags;
} probe_protect_settings_t;

static tool_data_t *current_tool;

static uint8_t probe_connect_port;
static bool nvs_hardlimits, nvs_invert_probe_pin = false;
static probe_connected_flags_t probe_connected;
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
static stepper_pulse_start_ptr stepper_pulse_start;
static spindle_set_state_ptr on_spindle_set_state = NULL;
static on_tool_selected_ptr on_tool_selected = NULL;

ISR_CODE static void set_connected (uint8_t irq_port, bool is_high)
{
    grbl.enqueue_realtime_command(CMD_PROBE_CONNECTED_TOGGLE);
}

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

static void on_pulse_start (stepper_t *stepper){

    probe_state_t probe = hal.probe.get_state();

    if (probe.triggered || !probe.connected) { // Check probe state.
        grbl.enqueue_realtime_command(CMD_RESET);
        report_message("PROBE PROTECTED!", Message_Warning);
    }
    
    if(stepper_pulse_start)
        stepper_pulse_start(stepper);
}

static void protection_on (void){

    stepper_pulse_start = hal.stepper.pulse_start;
    hal.stepper.pulse_start = on_pulse_start;
}

static void protection_off (void){
    
    hal.stepper.pulse_start = stepper_pulse_start;
    stepper_pulse_start = NULL;  //risk of null pointer error?
}

static bool probe_start (axes_signals_t axes, float *target, plan_line_data_t *pl_data){
    //if probe connected, de-activate protection at the start of a probing move machine will stop on activation
    bool status = true;
    protection_off();

    if(on_probe_start)
        status = on_probe_start(axes, target, pl_data);
    
    return status;
}

static void probe_completed (void){
    //if probe connected, re-activate protection.
    protection_on();

    //restore anything changed during tool probing.
    settings.probe.invert_probe_pin = nvs_invert_probe_pin;
    hal.limits.enable(settings.limits.flags.hard_enabled, nvs_hardlimits);  //restore hard limit settings.

    if(on_probe_completed)
        on_probe_completed();
}

// When called from "normal" probing tool is always NULL, when called from within
// a tool change sequence (M6) then tool is a pointer to the selected tool.
bool probe_fixture (tool_data_t *tool, bool at_g59_3, bool on)
{
    bool status = true;

    if(tool){ //are doing a tool change.

        //set polarity before probing the fixture.
        if(probe_protect_settings.flags.invert)
            settings.probe.invert_probe_pin = !nvs_invert_probe_pin;

        //set hard limits before probing the fixture.
        if(tool && !nvs_hardlimits && probe_protect_settings.flags.hardlimits){ //if the hard limits are not already enabled they need to be enabled.
            hal.limits.enable(settings.limits.flags.hard_enabled, true); // Change immediately. NOTE: Nice to have but could be problematic later.
        }
        hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
    } else{
        //restore settings (not sure if needed)
        //settings.probe.invert_probe_pin = nvs_invert_probe_pin;
        //hal.limits.enable(settings.limits.flags.hard_enabled, nvs_hardlimits);  //restore hard limit settings.
    }

    if(on_probe_fixture)
        status = on_probe_fixture(tool, at_g59_3, on);

    return status;
}

static void on_probe_connected_toggle(void){
    
    int val = 0;
    //if there is a pin, read it
    if(probe_protect_settings.flags.ext_pin){
        //hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
        val = hal.port.wait_on_input(Port_Digital, probe_connect_port, WaitMode_Immediate, 0.0f);//read the IO pin        
        if(val == 1)
            probe_connected.ext_pin = true;
        else
            probe_connected.ext_pin = false;

        if(probe_protect_settings.flags.ext_pin_inv)
            probe_connected.ext_pin = !probe_connected.ext_pin;
    }

    
    if(probe_connected.ext_pin)
        report_message("External Probe connected!", Message_Info);    

    if (probe_connected.t99)
        report_message("T99 Probe connected!", Message_Info);

    if(probe_connected.mcode)
            report_message("Mcode Probe connected!", Message_Info);

    if(probe_connected.toggle)
            report_message("Probe connect toggled on", Message_Info);

    
    if(probe_connected.value)
        protection_on();
    else{
        protection_off();
        report_message("Probe disconnected, protection off.", Message_Info);
    }

    if(probe_connected_toggle)
        probe_connected_toggle();

}

static void onSpindleSetState (spindle_state_t state, float rpm)
{
    //If the probe is connected and the spindle is turning on, alarm.
    if(probe_connected.value && (state.value !=0)){
        state.value = 0; //ensure spindle is off
        grbl.enqueue_realtime_command(CMD_RESET);
        report_message("PROBE IS IN SPINDLE!", Message_Warning);
    }

    on_spindle_set_state(state, rpm);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{   
    on_spindle_set_state = spindle->set_state;
    spindle->set_state = onSpindleSetState;

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static void onToolSelected (tool_data_t *tool)
{
    //if the tool is 99, set probe connected.
    current_tool = tool;

    if (tool->tool == 99)
        probe_connected.t99 = true;
    else
        probe_connected.t99 = false;
    
    on_probe_connected_toggle();

    if(on_tool_selected)
        on_tool_selected(tool);
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    if (state != STATE_CHECK_MODE)
      switch((uint16_t)gc_block->user_mcode) {

        case 401:
            if(!probe_connected.mcode){
                probe_connected.mcode = true;
                //enqueue probe connected symbol.
                grbl.enqueue_realtime_command(CMD_PROBE_CONNECTED_TOGGLE);
                hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
            }else
                report_message("Probe connected signal already asserted!", Message_Warning);
            break;

        case 402:
            if(probe_connected.mcode){
                probe_connected.mcode = false;
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
    settings.probe.invert_probe_pin = nvs_invert_probe_pin;
    hal.limits.enable(settings.limits.flags.hard_enabled, nvs_hardlimits);  //restore hard limit settings.
    //probe_connected.value = 0;  //seems like it is best for this to survive reset.
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
    { PROBE_PLUGIN_FIXTURE_INVERT_LIMIT_SETTING, Group_Probing, "Probe Protection Flags", NULL, Format_Bitfield, "Invert Tool Probe,Hard Limits, External Pin, Invert External Pin", NULL, NULL, Setting_NonCore, &probe_protect_settings.flags, NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t probe_protect_settings_descr[] = {
    { PROBE_PLUGIN_PORT_SETTING, "Aux input port number to use for probe connected control.\\n\\n"
                            "NOTE: A hard reset of the controller is required after changing this setting."
    },
    { PROBE_PLUGIN_FIXTURE_INVERT_LIMIT_SETTING, "Inversion setting for Probe signal during tool measurement.\\n"
                            "Enable hard limits during tool probe.\\n"
                            "Enable external pin input for probe connected signal.\\n"
                            "Invert external pin input for probe connected signal.\\n\\n"
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
    nvs_hardlimits = settings.limits.flags.hard_enabled;
    nvs_invert_probe_pin = settings.probe.invert_probe_pin;

    if(probe_protect_settings.flags.ext_pin){
        if(ioport_claim(Port_Digital, Port_Input, &probe_connect_port, "Probe Connected")) {

            memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        } else
            protocol_enqueue_rt_command(warning_no_port);

        //Try to register the interrupt handler.
        if(!(hal.port.register_interrupt_handler(probe_connect_port, IRQ_Mode_Change, set_connected)))
            protocol_enqueue_rt_command(warning_no_port);
    }


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
    probe_connected.value = 0;

    //Register function pointers
    probe_connected_toggle = hal.probe.connected_toggle;
    hal.probe.connected_toggle = on_probe_connected_toggle;

    on_probe_fixture = grbl.on_probe_fixture;
    grbl.on_probe_fixture = probe_fixture;

    on_probe_completed = grbl.on_probe_completed;
    grbl.on_probe_completed = probe_completed;

    on_probe_start = grbl.on_probe_start;
    grbl.on_probe_start = probe_start;

    on_spindle_select = grbl.on_spindle_select;
    grbl.on_spindle_select = onSpindleSelect;

    on_tool_selected = grbl.on_tool_selected;
    grbl.on_tool_selected = onToolSelected;

    driver_reset = hal.driver_reset;
    hal.driver_reset = probe_reset;

    //note that these do not chain.
    hal.user_mcode.check = mcode_check;
    hal.user_mcode.validate = mcode_validate;
    hal.user_mcode.execute = mcode_execute;   
    memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t)); 

    if(!ioport_can_claim_explicit()) {

        // Driver does not support explicit pin claiming, claim the highest numbered port instead.

        if((ok = hal.port.num_digital_in > 0)) {

            probe_connect_port = hal.port.num_digital_in - 1; //M62 can still be used.

            if(hal.port.set_pin_description)
                hal.port.set_pin_description(Port_Digital, Port_Input, probe_connect_port, "Probe detect implicit");

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

