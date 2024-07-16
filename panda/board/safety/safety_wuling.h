// CAN msgs we care about
//safety wuling
#define ENGINE_DATA   0xc9
#define LKAS_HUD      0x373
#define STEERING_LKAS      0x225
#define BRAKE_DATA      0x269
#define GAS_DATA      0x260

// CAN bus numbers
#define BUS_MAIN 0
#define BUS_RADAR  1
#define BUS_CAM  2

#define WULING_MAX_STEER 2048U

// max delta torque allowed for real time checks
#define WULING_MAX_RT_DELTA 940
// 250ms between real time checks
#define WULING_RT_INTERVAL 250000
#define WULING_MAX_RATE_UP 10
#define WULING_MAX_RATE_DOWN 25
#define WULING_DRIVER_TORQUE_ALLOWANCE 15
#define WULING_DRIVER_TORQUE_FACTOR 1
#define WULING_MAX_TORQUE_ERROR 350

const CanMsg WULING_TX_MSGS[] = {{ENGINE_DATA, 0, 8}, {LKAS_HUD, 0, 8}};

AddrCheckStruct wl_addr_checks[] = {
  {.msg = {{ENGINE_DATA, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{BRAKE_DATA, 0, 8, .expected_timestep = 50000U}, { 0 }, { 0 }}},
  {.msg = {{GAS_DATA, 0, 8, .expected_timestep = 50000U}, { 0 }, { 0 }}},
};

#define WL_RX_CHECK_LEN (sizeof(wl_addr_checks) / sizeof(wl_addr_checks[0]))
addr_checks wl_rx_checks = {wl_addr_checks, WL_RX_CHECK_LEN};
// track msgs coming from OP so that we know what CAM msgs to drop and what to forward

static int WULING_rx_hook(CANPacket_t *to_push) {
  bool valid = addr_safety_check(to_push, &wl_rx_checks, NULL, NULL, NULL);
  return valid;
}

static int WULING_tx_hook(CANPacket_t *to_send) {
  UNUSED(to_send);
  return 1;
}

static int WULING_fwd_hook(int bus, CANPacket_t *to_fwd) {
  // fwd from car to camera. also fwd certain msgs from camera to car
  UNUSED(to_fwd);
  int bus_fwd = -1;

  if (bus == BUS_MAIN) {
    bus_fwd = BUS_CAM;
  } else if (bus == BUS_CAM) {
    // bool block = (addr == LKAS_HUD) || (addr == STEERING_LKAS);
    // bool block =  (addr == STEERING_LKAS) || (addr == BRAKE_DATA)  || (addr == GAS_DATA);
    // if (!block) {
      bus_fwd = BUS_MAIN;
  
  } else {
    // don't fwd
  }

  return bus_fwd;
}

static const addr_checks* WULING_init(int16_t param) {
  UNUSED(param);
  controls_allowed = 1;
  relay_malfunction_reset();
  return &wl_rx_checks;
}

const safety_hooks wuling_hooks = {
  .init = WULING_init,
  .rx = WULING_rx_hook,
  .tx = WULING_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = WULING_fwd_hook,
};