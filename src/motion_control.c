/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Simen Svale Skogsrud
  
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
*/

#include "grbl.h"


// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
// NOTE: This is the primary gateway to the grbl planner. All line motions, including arc line 
// segments, must pass through this routine before being passed to the planner. The seperation of
// mc_line and plan_buffer_line is done primarily to place non-planner-type functions from being
// in the planner and to let backlash compensation or canned cycle integration simple and direct.
#ifdef USE_LINE_NUMBERS
  void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number)
#else
  void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate)
#endif
{
  // If enabled, check for soft limit violations. Placed here all line motions are picked up
  // from everywhere in Grbl.
  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) { limits_soft_check(target); }    
      
  // If in check gcode mode, prevent motion by blocking planner. Soft limits still work.
  if (sys.state == STATE_CHECK_MODE) { return; }
    
  // NOTE: Backlash compensation may be installed here. It will need direction info to track when
  // to insert a backlash line motion(s) before the intended line motion and will require its own
  // plan_check_full_buffer() and check for system abort loop. Also for position reporting 
  // backlash steps will need to be also tracked, which will need to be kept at a system level.
  // There are likely some other things that will need to be tracked as well. However, we feel
  // that backlash compensation should NOT be handled by Grbl itself, because there are a myriad
  // of ways to implement it and can be effective or ineffective for different CNC machines. This
  // would be better handled by the interface as a post-processor task, where the original g-code
  // is translated and inserts backlash motions that best suits the machine. 
  // NOTE: Perhaps as a middle-ground, all that needs to be sent is a flag or special command that
  // indicates to Grbl what is a backlash compensation motion, so that Grbl executes the move but
  // doesn't update the machine position values. Since the position values used by the g-code
  // parser and planner are separate from the system machine positions, this is doable.

  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Remain in this loop until there is room in the buffer.
  do {
    protocol_execute_realtime(); // Check for any run-time commands
    if (sys.abort) { return; } // Bail, if system abort.
    if ( plan_check_full_buffer() ) { protocol_auto_cycle_start(); } // Auto-cycle start when buffer is full.
    else { break; }
  } while (1);

  // Plan and queue motion into planner buffer
  #ifdef USE_LINE_NUMBERS
    plan_buffer_line(target, feed_rate, invert_feed_rate, line_number);
  #else
    plan_buffer_line(target, feed_rate, invert_feed_rate);
  #endif
}

// Execute dwell in seconds.
void mc_dwell(float seconds) 
{
   if (sys.state == STATE_CHECK_MODE) { return; }
   
   uint16_t i = floor(1000/DWELL_TIME_STEP*seconds);
   protocol_buffer_synchronize();
   delay_ms(floor(1000*seconds-i*DWELL_TIME_STEP)); // Delay millisecond remainder.
   while (i-- > 0) {
     // NOTE: Check and execute realtime commands during dwell every <= DWELL_TIME_STEP milliseconds.
     protocol_execute_realtime();
     if (sys.abort) { return; }
     _delay_ms(DWELL_TIME_STEP); // Delay DWELL_TIME_STEP increment
   }
}


// Perform homing cycle to locate and set machine zero. Only '$H' executes this command.
// NOTE: There should be no motions in the buffer and Grbl must be in an idle state before
// executing the homing cycle. This prevents incorrect buffered plans after homing.
void mc_homing_cycle()
{
  // Check and abort homing cycle, if hard limits are already enabled. Helps prevent problems
  // with machines with limits wired on both ends of travel to one limit pin.
  // TODO: Move the pin-specific LIMIT_PIN call to limits.c as a function.
  #ifdef LIMITS_TWO_SWITCHES_ON_AXES  
    if (limits_get_state()) { 
      mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
      bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT));
      return;
    }
  #endif
   
  limits_disable(); // Disable hard limits pin change register for cycle duration
    
  // -------------------------------------------------------------------------------------
  // Perform homing routine. NOTE: Special motion case. Only system reset works.
  
  // Search to engage all axes limit switches at faster homing seek rate.
  limits_go_home(HOMING_CYCLE_0);  // Homing cycle 0
  #ifdef HOMING_CYCLE_1
    limits_go_home(HOMING_CYCLE_1);  // Homing cycle 1
  #endif
  #ifdef HOMING_CYCLE_2
    limits_go_home(HOMING_CYCLE_2);  // Homing cycle 2
  #endif
    
  protocol_execute_realtime(); // Check for reset and set system abort.
  if (sys.abort) { return; } // Did not complete. Alarm state set by mc_alarm.

  // Homing cycle complete! Setup system for normal operation.
  // -------------------------------------------------------------------------------------

  // Gcode parser position was circumvented by the limits_go_home() routine, so sync position now.
  gc_sync_position();

  // If hard limits feature enabled, re-enable hard limits pin change register after homing cycle.
  limits_init();
}


// Perform tool length probe cycle. Requires probe switch.
// NOTE: Upon probe failure, the program will be stopped and placed into ALARM state.
#ifdef USE_LINE_NUMBERS
  void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away, 
    uint8_t is_no_error, int32_t line_number)
#else
  void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away,
    uint8_t is_no_error)
#endif
{ 
  // TODO: Need to update this cycle so it obeys a non-auto cycle start.
  if (sys.state == STATE_CHECK_MODE) { return; }

  // Finish all queued commands and empty planner buffer before starting probe cycle.
  protocol_buffer_synchronize();

  // Initialize probing control variables
  sys.probe_succeeded = false; // Re-initialize probe history before beginning cycle.  
  probe_configure_invert_mask(is_probe_away);
  
  // After syncing, check if probe is already triggered. If so, halt and issue alarm.
  // NOTE: This probe initialization error applies to all probing cycles.
  if ( probe_get_state() ) { // Check probe pin state.
    bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_PROBE_FAIL);
    protocol_execute_realtime();
  }
  if (sys.abort) { return; } // Return if system reset has been issued.

  // Setup and queue probing motion. Auto cycle-start should not start the cycle.
  #ifdef USE_LINE_NUMBERS
    mc_line(target, feed_rate, invert_feed_rate, line_number);
  #else
    mc_line(target, feed_rate, invert_feed_rate);
  #endif
  
  // Activate the probing state monitor in the stepper module.
  sys_probe_state = PROBE_ACTIVE;

  // Perform probing cycle. Wait here until probe is triggered or motion completes.
  bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START);
  do {
    protocol_execute_realtime(); 
    if (sys.abort) { return; } // Check for system abort
  } while (sys.state != STATE_IDLE);
  
  // Probing cycle complete!
  
  // Set state variables and error out, if the probe failed and cycle with error is enabled.
  if (sys_probe_state == PROBE_ACTIVE) {
    if (is_no_error) { memcpy(sys.probe_position, sys.position, sizeof(float)*N_AXIS); }
    else { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_PROBE_FAIL); }
  } else { 
    sys.probe_succeeded = true; // Indicate to system the probing cycle completed successfully.
  }
  sys_probe_state = PROBE_OFF; // Ensure probe state monitor is disabled.
  protocol_execute_realtime();   // Check and execute run-time commands
  if (sys.abort) { return; } // Check for system abort

  // Reset the stepper and planner buffers to remove the remainder of the probe motion.
  st_reset(); // Reest step segment buffer.
  plan_reset(); // Reset planner buffer. Zero planner positions. Ensure probing motion is cleared.
  plan_sync_position(); // Sync planner position to current machine position.

  // TODO: Update the g-code parser code to not require this target calculation but uses a gc_sync_position() call.
  // NOTE: The target[] variable updated here will be sent back and synced with the g-code parser.
  system_convert_array_steps_to_mpos(target, sys.position);

  #ifdef MESSAGE_PROBE_COORDINATES
    // All done! Output the probe position as message.
    report_probe_parameters();
  #endif
}


// Method to ready the system to reset by setting the realtime reset command and killing any
// active processes in the system. This also checks if a system reset is issued while Grbl
// is in a motion state. If so, kills the steppers and sets the system alarm to flag position
// lost, since there was an abrupt uncontrolled deceleration. Called at an interrupt level by
// realtime abort command and hard limits. So, keep to a minimum.
void mc_reset()
{
  // Only this function can set the system reset. Helps prevent multiple kill calls.
  if (bit_isfalse(sys_rt_exec_state, EXEC_RESET)) {
    bit_true_atomic(sys_rt_exec_state, EXEC_RESET);

    // Kill spindle and coolant.   
    spindle_stop();
    coolant_stop();

    // Kill steppers only if in any motion state, i.e. cycle, actively holding, or homing.
    // NOTE: If steppers are kept enabled via the step idle delay setting, this also keeps
    // the steppers enabled by avoiding the go_idle call altogether, unless the motion state is
    // violated, by which, all bets are off.
    if ((sys.state & (STATE_CYCLE | STATE_HOMING)) || (sys.suspend == SUSPEND_ENABLE_HOLD)) {
      if (sys.state == STATE_HOMING) { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_HOMING_FAIL); }
      else { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_ABORT_CYCLE); }
      st_go_idle(); // Force kill steppers. Position has likely been lost.
    }
  }
}
