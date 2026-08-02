// controls servo angle (lock)
bool servo_lock(int target_pos){ 
  if (pos == target_pos){
    return true;
  } 

  // write in increments
  pos = (target_pos > pos)? (pos + 1):(pos - 1);
  servo.write(pos);
  return false;
}

void calc_display_times(unsigned long input_time_s){
  time_disp_hrs = (input_time_s / 60) / 60;
  time_disp_mins = (input_time_s / 60) % 60;
  time_disp_s = input_time_s % 60;
}

void reset_all_timer_vars(){
  lock_time_s = 0;
  lock_time_start = 0;
  time_left = 0;
  time_disp_s = 0;
  time_disp_mins = 0;
  time_disp_hrs = 0;
  elapsed = 0;
}
