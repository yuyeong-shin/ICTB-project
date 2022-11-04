
#ifndef ID_OPEN_H
#define ID_OPEN_H

/*
 *  Enabling both WiFi and Bluetooth will almost certainly require a partition scheme 
 *  with > 1.2M for the application.
 */

#define ID_OD_WIFI_NAN    1
#define ID_OD_WIFI_BEACON 1
#define ID_OD_BT          0        // ASTM F3411-19 / ASD-STAN 4709-002.
#define BLE_SERVICES      0        // Experimental.

#if ID_OD_WIFI_NAN || ID_OD_WIFI_BEACON
#define ID_OD_WIFI        1
#else
#define ID_OD_WIFI        0
#endif

#define USE_BEACON_FUNC   0
#define WIFI_CHANNEL      6        // Be carefull changing this.
#define BEACON_FRAME_SIZE 256

#define ID_OD_AUTH_DATUM  1546300800LU



#if ID_OD_BT
//#include "BLEDevice.h"
//#include "BLEUtils.h"

#endif

#include "utm.h"

#include "opendroneid.h"


void ID_OpenDrone_init(void);
void ID_OpenDrone_param_init(struct UTM_parameters *parameters);
void ID_OpenDrone_set_auth(char *auth);
void ID_OpenDrone_set_auth_set(uint8_t *auth, short int len, uint8_t type);
int ID_OpenDrone_transmit(struct UTM_data *utm_data);

int ID_OpenDrone_transmit_wifi(struct UTM_data *utm_data);
int ID_OpenDrone_transmit_ble(uint8_t *odid_msg, int length);

void ID_OpenDrone_utm_message_pack(ODID_UAS_Data *outdata, struct UTM_data *indata);

#endif