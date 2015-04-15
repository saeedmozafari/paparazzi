inline static void h_ctl_auto_rudder_loop(void)
{
  static float last_err;

  float f_rudder = 0;
  float err = *stateGetHorizontalSpeedDir_f() - h_ctl_course_setpoint;
  NormRadAngle(err);
  float d_err = err - last_err;
  last_err = err;
  float controlled_rudder = - h_ctl_auto_rudder_pgain *
                              (err + v_ctl_auto_throttle_igain * v_ctl_auto_throttle_sum_err
                               + v_ctl_auto_throttle_dgain * d_err);

  /* pitch pre-command */
  f_rudder = controlled_rudder;
  h_ctl_auto_rudder_sum_err += err;
  BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR);
  v_ctl_pitch_setpoint = v_ctl_pitch_of_vz + v_ctl_pitch_trim + nav_pitch;
#if defined AGR_CLIMB
      break;
  } /* switch submode */
#endif

  v_ctl_throttle_setpoint = TRIM_UPPRZ(f_throttle * MAX_PPRZ);
}