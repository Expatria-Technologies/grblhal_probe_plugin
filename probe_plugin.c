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
#define PROBE_DEBOUNCE 25 // ms - increase if probe is slow and/or bouncy

#define PROBE_PLUGIN_PORT_SETTING1 Setting_UserDefined_7
#define PROBE_PLUGIN_PORT_SETTING2 Setting_UserDefined_8
#define PROBE_PLUGIN_FIXTURE_INVERT_LIMIT_SETTING Setting_UserDefined_9



//add function pointers for tool number and pulse start


static uint8_t n_ports;
static char max_port[4];

typedef union {
    uint8_t value;
    struct {
        uint8_t
        invert         :1,
        ext_pin        :1,
        ext_pin_inv    :1,
        tool_pin       :1,
        tool_pin_inv   :1,
        motion_protect :1,
        t99_protect    :1,      
        reserved       :1;
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
    uint8_t protect_port;
    uint8_t tool_port;
    probe_protect_flags_t flags;
    uint16_t debounce;
} probe_protect_settings_t;

//static probe_state_t probe = {
//    .connected = Off
//};

static tool_data_t *current_tool;

static probe_state_t prev_probe;

static uint8_t probe_connect_port;
static uint8_t tool_probe_port;
static bool nvs_invert_probe_pin, protection_enabled = false;
static uint8_t nvs_hardlimits;
static probe_connected_flags_t probe_connected;
static driver_reset_ptr driver_reset;
static user_mcode_ptrs_t user_mcode;

static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
//static probe_connected_toggle_ptr probe_connected_toggle;
static probe_protect_settings_t probe_protect_settings;
static on_probe_start_ptr on_probe_start;
static on_probe_completed_ptr on_probe_completed;
static on_probe_toolsetter_ptr on_probe_fixture;
static on_spindle_select_ptr on_spindle_select;
static stepper_pulse_start_ptr stepper_pulse_start;
static spindle_set_state_ptr on_spindle_set_state = NULL;
static on_tool_selected_ptr on_tool_selected = NULL;
static on_tool_changed_ptr on_tool_changed = NULL; 
static probe_get_state_ptr probe_get_state = NULL;
static probe_configure_ptr on_probe_configure = NULL;

static void set_connected_status(uint_fast16_t state);

//returns true if probe is connected and sets core variable.
static void check_connected_pin (void)
{
    uint8_t val;

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
}

ISR_CODE static void set_connected (uint8_t irq_port, bool is_high)
{
    protocol_enqueue_rt_command(set_connected_status);    
}

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)401 || mcode == (user_mcode_t)402
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
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

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

// local redirected probing function for tool probe pin.
static probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = On; //tool setter is fixed and always connected.  Maybe add some error handling here?
    state.triggered = hal.port.wait_on_input(Port_Digital, tool_probe_port, WaitMode_Immediate, 0.0f);//read the IO pin

    if(probe_protect_settings.flags.tool_pin_inv)
        state.triggered = !state.triggered;   

    return state;
}

//called after short delay to verify that a rising edge on the probe pin is still asserted
static void protect_debounce_cb(void){
    probe_state_t probe = hal.probe.get_state();

    if (probe.triggered) { // Check probe state.  Alarm if still asserted.
        grbl.enqueue_realtime_command(CMD_STOP);
        report_message("PROBE PROTECTED!", Message_Warning);
    }    
}

static void on_pulse_start (stepper_t *stepper){

    probe_state_t probe = hal.probe.get_state();

    //if ((!prev_probe.triggered && probe.triggered) || !probe.connected) { // if probe has a rising edge from last pulse or is disconnected.
    if ((!prev_probe.triggered && probe.triggered)) { // if probe has a rising edge from last pulse
        //call debounce callback
        hal.delay_ms(PROBE_DEBOUNCE, protect_debounce_cb);
    }

    prev_probe.triggered = probe.triggered;
    
    if(stepper_pulse_start)
        stepper_pulse_start(stepper);
}

static void protection_on (void){

    if (!protection_enabled && probe_connected.value && probe_protect_settings.flags.motion_protect){
        
        protection_enabled = On;

        probe_state_t probe = hal.probe.get_state();
        prev_probe.triggered = probe.triggered;

        stepper_pulse_start = hal.stepper.pulse_start;
        hal.stepper.pulse_start = on_pulse_start;   
        //report_message("Protection On", Message_Info);             
    }
}

static void protection_off (void){
    
    if(protection_enabled && probe_protect_settings.flags.motion_protect){
        protection_enabled = Off;

        if(stepper_pulse_start)
            hal.stepper.pulse_start = stepper_pulse_start;
        stepper_pulse_start = NULL;  //risk of null pointer error?
        //report_message("Protection Off", Message_Info);        
    }
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
    //re-activate protection.
    protection_on();

    if(on_probe_completed)
        on_probe_completed();
}

static void tool_changed (tool_data_t *tool){    
    //tool_change.c sets grbl.on_probe_completed to NULL when finished.  Is this correct?  That breaks the call chain in this plugin.
    //restore the pointer here.
    
    if(grbl.on_probe_completed == NULL){
        on_probe_completed = grbl.on_probe_completed;
        grbl.on_probe_completed = probe_completed;
    }

    //continue call chain
    if(on_tool_changed)
        on_tool_changed(tool);
} 

//The grbl.on_probe_fixture event handler is called by the default tool change algorithm when probing at G59.3.
//In addition it will be called on a "normal" probe sequence if the XY position is
//within the radius of the G59.3 position defined below.
// When called from "normal" probing tool is always NULL, when called from within
// a tool change sequence (M6) then tool is a pointer to the selected tool.
bool probe_fixture (tool_data_t *tool, bool at_g59_3, bool on)
{
    bool status = true;

    if(at_g59_3 && on){ //are doing a tool change.
        
        report_message("Doing tool probe", Message_Info);

        protection_off();  //disable protection when probing

        //set polarity before probing the fixture.
        if(probe_protect_settings.flags.invert)
            settings.probe.invert_probe_pin = !nvs_invert_probe_pin;
        
        //if a different pin is configured, re-direct probe reading to that pin via function pointer.
        if(probe_protect_settings.flags.tool_pin){
            report_message("Activating alternate tool pin", Message_Info);
            //store current probe state function
            if(hal.probe.get_state)
                probe_get_state = hal.probe.get_state;
            hal.probe.get_state = probeGetState;
        }
        //set hard limits before probing the fixture.
        //if(!nvs_hardlimits && probe_protect_settings.flags.hardlimits){ //if the hard limits are not already enabled they need to be enabled.
        //    hal.limits.enable(settings.limits.flags.hard_enabled, true); // Change immediately. NOTE: Nice to have but could be problematic later.
        //}
    }else{
        if(probe_protect_settings.flags.tool_pin){
            report_message("Restoring probe pin", Message_Info);
            //restore probe state function
            if(probe_get_state)
                hal.probe.get_state = probe_get_state;
            probe_get_state = NULL;
        }
        if(probe_protect_settings.flags.invert)                
            settings.probe.invert_probe_pin = nvs_invert_probe_pin;  //restore pin inversion setting
        //hal.limits.enable(settings.limits.flags.hard_enabled, nvs_hardlimits);  //restore hard limit settings.
        protection_on();      //restore protection.  
    }
    
    //typedef bool (*on_probe_toolsetter_ptr)(tool_data_t *tool, coord_data_t *position, bool at_g59_3, bool on)
    if(on_probe_fixture)
        status = on_probe_fixture(tool, NULL, at_g59_3, on);

    return status;
}

//static void on_probe_connected_toggle(void){
static void set_connected_status(uint_fast16_t state){    
    
    static uint8_t previous_flags;
    //probe_state_t probe = hal.probe.get_state();

    check_connected_pin();

    if(probe_connected.ext_pin)
        report_message("External Probe connected!", Message_Info);    

    if (probe_connected.t99)
        report_message("T99 Probe connected!", Message_Info);

    if(probe_connected.mcode)
            report_message("Mcode Probe connected!", Message_Info);

    if(probe_connected.toggle)
            report_message("Probe connect toggled on", Message_Info);
    
    if(probe_connected.value){
        protection_on(); 
    } else{
        protection_off(); 
        if (previous_flags != probe_connected.value)
            report_message("Probe disconnected, protection off.", Message_Info);
    }

    previous_flags = probe_connected.value;   
}

static void onSpindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    //If the probe is connected and the spindle is turning on, alarm.
    if(probe_connected.value && (state.value !=0)){
        state.value = 0; //ensure spindle is off
        grbl.enqueue_realtime_command(CMD_STOP);
        report_message("PROBE IS IN SPINDLE!", Message_Warning);
    }

    on_spindle_set_state(spindle, state, rpm);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{   
    on_spindle_set_state = spindle->set_state;
    spindle->set_state = onSpindleSetState;

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static void onToolSelected (tool_data_t *tool)
{
    //probe_state_t probe = hal.probe.get_state();
    //if the tool is 99, set probe connected.
    current_tool = tool;

    if ((tool->tool_id == 99) && probe_protect_settings.flags.t99_protect){
        probe_connected.t99 = true;     
    }else {
        probe_connected.t99 = false;        
    }

    set_connected_status(tool->tool_id);

    if(on_tool_selected)
        on_tool_selected(tool);
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;
    //probe_state_t probe = hal.probe.get_state();

    if (state != STATE_CHECK_MODE)
      switch((uint16_t)gc_block->user_mcode) {

        case 401:
            if(!probe_connected.mcode){
                probe_connected.mcode = true;               
            }else
                report_message("Probe connected signal already asserted!", Message_Warning);
            break;

        case 402:
            if(probe_connected.mcode){
                probe_connected.mcode = false;
            }else
                report_message("Probe connected signal not asserted!", Message_Warning);
            break;

        default:
            handled = false;
            break;
    }

    set_connected_status(probe_connected.mcode);  

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void probeConfigure (bool is_probe_away, bool probing)
{
    if(on_probe_configure)
        on_probe_configure(is_probe_away, probing);
    
}

static void probe_reset (void)
{
    //settings.probe.invert_probe_pin = nvs_invert_probe_pin;
    if(probe_get_state){
        hal.probe.get_state = probe_get_state;
        probe_get_state = NULL;
    }   
    hal.limits.enable(settings.limits.flags.hard_enabled, (axes_signals_t)nvs_hardlimits);  //restore hard limit settings.
    //probe_connected.value = 0;  //seems like it is best for this to survive reset.
    //protocol_enqueue_rt_command(set_connected_status);
    probe_state_t probe = hal.probe.get_state();
    if (!probe.connected)
        grbl.enqueue_realtime_command(CMD_PROBE_CONNECTED_TOGGLE);      
    driver_reset();
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:Probe Protection v0.01]" ASCII_EOL);
    }       
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
    { PROBE_PLUGIN_PORT_SETTING1, Group_Probing, "Probe Connected Aux Input", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &probe_protect_settings.protect_port, NULL, NULL },
    { PROBE_PLUGIN_PORT_SETTING2, Group_Probing, "Tool Probe Aux Input", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &probe_protect_settings.tool_port, NULL, NULL },    
    { PROBE_PLUGIN_FIXTURE_INVERT_LIMIT_SETTING, Group_Probing, "Probe Protection Flags", NULL, Format_Bitfield, "Invert Tool Probe, External Connected Pin, Invert External Connected Pin, Alternate Tool Probe Pin, Invert Tool Probe Pin, Enable Motion Protection, T99 Probe Connected", NULL, NULL, Setting_NonCore, &probe_protect_settings.flags, NULL, NULL },   
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t probe_protect_settings_descr[] = {
    { PROBE_PLUGIN_PORT_SETTING1, "Aux input port number to use for probe connected control.\\n\\n"
                            "NOTE: A hard reset of the controller is required after changing this setting."
    },
    { PROBE_PLUGIN_PORT_SETTING2, "Aux input port number to use for tool probing at G59.3.\\n\\n"
                            "NOTE: A hard reset of the controller is required after changing this setting."
    },    
    { PROBE_PLUGIN_FIXTURE_INVERT_LIMIT_SETTING, "Inversion setting for Probe signal during tool measurement.\\n"
                            "Enable external pin input for probe connected signal.\\n"
                            "Invert external pin input for probe connected signal.\\n"
                            "Enable alternate pin input for Tool Probe signal.\\n"
                            "Invert alternate pin input for Tool Probe signal.\\n"    
                            "Enable probe motion protection.  Alarm will trip if probe is asserted on non-probing moves (Experimental).\\n"
                            "Enable probe protection on T99.  Spindle is disabled when T99 (probe) is selected.\\n"                          
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
    probe_protect_settings.protect_port = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    probe_protect_settings.tool_port = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    probe_protect_settings.flags.value = 0;
    probe_protect_settings.flags.t99_protect = 1;

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
    if(probe_protect_settings.protect_port >= n_ports)
        probe_protect_settings.protect_port = n_ports - 1;

    if(probe_protect_settings.tool_port >= n_ports)
        probe_protect_settings.tool_port = n_ports - 2;        

    probe_connect_port = probe_protect_settings.protect_port;
    tool_probe_port = probe_protect_settings.tool_port;
    nvs_hardlimits = settings.limits.flags.hard_enabled;
    nvs_invert_probe_pin = settings.probe.invert_probe_pin;

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    if(probe_protect_settings.flags.ext_pin){
        if(ioport_claim(Port_Digital, Port_Input, &probe_connect_port, "Probe Connected")) {
        } else
            protocol_enqueue_rt_command(warning_no_port);    

        //Try to register the interrupt handler.
        if(!(hal.port.register_interrupt_handler(probe_connect_port, IRQ_Mode_Change, set_connected)))
            protocol_enqueue_rt_command(warning_no_port);
    }

    if(probe_protect_settings.flags.tool_pin){
        if(ioport_claim(Port_Digital, Port_Input, &tool_probe_port, "Toolsetter G59.3")) {
        } else
            protocol_enqueue_rt_command(warning_no_port);    
        //Not an interrupt pin.
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
    bool ok = (n_ports = ioports_available(Port_Digital, Port_Input));
    probe_connected.value = 0;

    //Register function pointers
    on_tool_changed = grbl.on_tool_changed;
    grbl.on_tool_changed = tool_changed;

    on_probe_fixture = grbl.on_probe_toolsetter;
    grbl.on_probe_toolsetter = probe_fixture;

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
    
    on_probe_configure = hal.probe.configure;
    hal.probe.configure = probeConfigure;

    //note that these do not chain.
    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;   
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t)); 

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

    } else if((ok = (nvs_address = nvs_alloc(sizeof(probe_protect_settings_t))))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        settings_register(&setting_details);

        // Used for setting value validation
        strcpy(max_port, uitoa(n_ports - 1));
    }

    //need to add some init code to check the pin states (if enabled) and set the connected flag on or off.
    probe_state_t probe = hal.probe.get_state();
    if (!probe.connected)
        grbl.enqueue_realtime_command(CMD_PROBE_CONNECTED_TOGGLE);  

    if(!ok)
        protocol_enqueue_rt_command(warning_msg);
}

