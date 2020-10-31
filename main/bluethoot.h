#ifndef BT_H_
#define BT_H_

#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"


void bt_initialize(void);
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void bt_send_data(uint8_t *p_data, uint16_t len);

#endif /* end of include guard: _BT_HPP_ */
