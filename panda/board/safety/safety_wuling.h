// int wuling_rx_hook(CANPacket_t *to_push) {
//   UNUSED(to_push);
//   controls_allowed = 1;
//   return true;
// }

// static int wuling_tx_hook(CANPacket_t *to_send) {
//   UNUSED(to_send);
//   return true;
// }


// static int wuling_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
//   UNUSED(to_fwd);
//   int bus_fwd = -1;
  
//   if (bus_num == 0) {
//     bus_fwd = 2;
//   }

//   if (bus_num == 2) {
//     bus_fwd = 0;
//   }

//   return bus_fwd;
// }

const safety_hooks wuling_hooks = {
  // .init = nooutput_init,
  // .rx = wuling_rx_hook,
  // .tx = wuling_tx_hook,
  // .tx_lin = nooutput_tx_lin_hook,
  // .fwd = wuling_fwd_hook,
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = mazda_fwd_hook,
};
