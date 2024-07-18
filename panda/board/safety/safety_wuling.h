
// CAN msgs we care about
//safety wuling
#define ENGINE_DATA   0xc9
#define LKAS_HUD      0x373
#define STEERING_LKAS      0x225
#define BRAKE_DATA      0x269
#define GAS_DATA      0x260

#define ACC_STS      0x263
#define CRZ_CTRL      0x370

// CAN bus numbers
#define BUS_MAIN 0
#define BUS_RADAR  1
#define BUS_CAM  2

const CanMsg WULING_TX_MSGS[] = {{ENGINE_DATA, 0, 8}, {LKAS_HUD, 0, 8}};

AddrCheckStruct wl_addr_checks[] = {
  {.msg = {{ENGINE_DATA, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{BRAKE_DATA, 0, 8, .expected_timestep = 50000U}, { 0 }, { 0 }}},
  {.msg = {{GAS_DATA, 0, 8, .expected_timestep = 50000U}, { 0 }, { 0 }}},
  {.msg = {{LKAS_HUD, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{ACC_STS, 0, 8, .expected_timestep = 50000U}, { 0 }, { 0 }}},
  {.msg = {{CRZ_CTRL, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};

#define WL_RX_CHECK_LEN (sizeof(wl_addr_checks) / sizeof(wl_addr_checks[0]))
addr_checks wl_rx_checks = {wl_addr_checks, WL_RX_CHECK_LEN};

static const addr_checks* wuling_init(int16_t param) {
  UNUSED(param);
  controls_allowed = 1;
  relay_malfunction_reset();
  return &wl_rx_checks;
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static int wuling_rx_hook(CANPacket_t *to_push) {


  bool valid = addr_safety_check(to_push, &wl_rx_checks, NULL, NULL, NULL);

   if (valid && ((int)GET_BUS(to_push) == BUS_MAIN)) {
      int addr = GET_ADDR(to_push);

      // 840 -> 348 (Wheel Speed)
      if (addr == 840) {
        vehicle_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);
      }

      // 485 -> 1e5 (Steering Angle)
      if (addr == 485) {
        int torque_driver_new = GET_BYTE(to_push, 6);
        // update array of samples
        update_sample(&torque_driver, torque_driver_new);
      }
      // 201 -> c9 (ECMEngineStatus)
      if (addr == 201) {
        brake_pressed = GET_BIT(to_push, 40U) != 0U;
      }

      if (addr == 0x191) {
        gas_pressed = GET_BYTE(to_push, 6) != 0U;
      }

      // generic_rx_checks((addr == LKAS_HUD));
   }

  controls_allowed = 1;
  
  return true;
}

// all commands: gas, brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation
// Message send to main bus

static int wuling_tx_hook(CANPacket_t *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);
  UNUSED(addr);
  UNUSED(bus);
  controls_allowed = 1;

  // 1 allows the message through
  return tx;
}

static int wuling_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  // fwd from car to camera. also fwd certain msgs from camera to car
 
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  if (bus_num == BUS_MAIN) {
    bus_fwd = BUS_CAM;
  } else if (bus_num == BUS_CAM) {
    bool block =  (addr == STEERING_LKAS);
    if (!block) {
      bus_fwd = BUS_MAIN;
    }
 
  } else {
    // don't fwd
  }

  return bus_fwd;
}

const safety_hooks wuling_hooks = {
  .init = wuling_init,
  .rx = wuling_rx_hook,
  .tx = wuling_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = wuling_fwd_hook,
};