//TODO: add code for your component here
#pragma GCC diagnostic warning "-Wunused-variable"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>

#include "id_open.h"
#include "utm.h"
#include "opendroneid.h"


#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"

#include "sdkconfig.h"

#include "lwip/opt.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"

struct struct_Position {
	double latitude;
	double longitude;
	double x;
	double y;
	double heading;
	double speed;
} ;

unsigned char id = 0;

void  GPS_gp2utm(double sphi, double slam, int *izone, double *y, double *x);
void GPS_utm2gp(double *sphi, double *slam, int izone, double y, double x);
struct struct_Position circle_path(unsigned char id, double lon, double lat, double radius);
struct struct_Position sPosition[2];
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
/*
// local function
static wifi_mode_t getMode(void);
static bool wifiLowLevelInit(bool persistent);
static esp_netif_t* esp_netifs[ESP_IF_MAX] = {NULL, NULL, NULL};
static bool tcpipInit(void);
*/

// local variables
int                     auth_page = 0, auth_page_count = 0;
char                   *UAS_operator;
uint8_t                 msg_counter[16];

#if ID_OD_WIFI
char                    ssid[32];
uint8_t                 WiFi_mac_addr[6], wifi_channel;
size_t                  ssid_length = 0;
#if ID_OD_WIFI_BEACON
int                     beacon_offset = 0, beacon_max_packed = 30;
uint8_t                 beacon_frame[BEACON_FRAME_SIZE],
#if USE_BEACON_FUNC
                    beacon_counter = 0;
#else
*beacon_payload, *beacon_timestamp, *beacon_counter, *beacon_length;
#endif
#endif
#endif

#if ID_OD_BT
uint8_t                 ble_message[36], counter = 0;
int                     advertising = 0;
esp_ble_adv_data_t      advData;
esp_ble_adv_params_t    advParams;
BLEUUID                 service_uuid;
#if BLE_SERVICES
BLEServer              *ble_server = NULL;
BLEService             *ble_service_dbm = NULL;
BLECharacteristic      *ble_char_dbm = NULL;
#endif
#endif

ODID_UAS_Data           UAS_data;
ODID_BasicID_data      *basicID_data;
ODID_Location_data     *location_data;
ODID_Auth_data         *auth_data[ODID_AUTH_MAX_PAGES];
ODID_SelfID_data       *selfID_data;
ODID_System_data       *system_data;
ODID_OperatorID_data   *operatorID_data;

ODID_BasicID_encoded    basicID_enc;
ODID_Location_encoded   location_enc;
ODID_Auth_encoded       auth_enc;
ODID_SelfID_encoded     selfID_enc;
ODID_System_encoded     system_enc;
ODID_OperatorID_encoded operatorID_enc;

// end of local variables
/*
// local variables
static bool _persistent = true;
static bool lowLevelInitDone = false;
*/
// end of local variables
void ID_OpenDrone_init(void)
{
	int i;
	static const char *dummy = "";

	UAS_operator = (char *)dummy;

#if ID_OD_WIFI

	wifi_channel = WIFI_CHANNEL;

	memset(WiFi_mac_addr, 0, 6);
	memset(ssid, 0, sizeof(ssid));

	strcpy(ssid, "UAS_ID_OPEN");

#if ID_OD_WIFI_BEACON

#if ODID_PACK_MAX_MESSAGES > 9
#undef ODID_PACK_MAX_MESSAGES
#define ODID_PACK_MAX_MESSAGES 9
#endif

	memset(beacon_frame, 0, BEACON_FRAME_SIZE);

#if !USE_BEACON_FUNC

	beacon_counter = 0;
	beacon_length = 0;
	beacon_timestamp = 0;
	beacon_payload = beacon_frame;

#endif  //!USE_BEACON_FUNC
#endif  //ID_OD_WIFI_BEACON
#endif  //ID_OD_WIFI

#if ID_OD_BT

	memset(&advData, 0, sizeof(advData));

	advData.set_scan_rsp        = false;
	advData.include_name        = false;
	advData.include_txpower     = false;
	advData.min_interval        = 0x0006;
	advData.max_interval        = 0x0050;
	advData.flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

	memset(&advParams, 0, sizeof(advParams));

	advParams.adv_int_min       = 0x0020;
	advParams.adv_int_max       = 0x0040;
	advParams.adv_type          = ADV_TYPE_IND;
	advParams.own_addr_type     = BLE_ADDR_TYPE_PUBLIC;
	advParams.channel_map       = ADV_CHNL_ALL;
	advParams.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
	advParams.peer_addr_type    = BLE_ADDR_TYPE_PUBLIC;

	service_uuid = BLEUUID("0000fffa-0000-1000-8000-00805f9b34fb");

#endif  //ID_OD_BT

	memset(msg_counter, 0, sizeof(msg_counter));

	//
	// Below '// 0' indicates where we are setting 0 to 0 for clarity.
	//

	memset(&UAS_data, 0, sizeof(ODID_UAS_Data));

	basicID_data    = &UAS_data.BasicID[0];
	location_data   = &UAS_data.Location;
	selfID_data     = &UAS_data.SelfID;
	system_data     = &UAS_data.System;
	operatorID_data = &UAS_data.OperatorID;

	for (i = 0; i < ODID_AUTH_MAX_PAGES; ++i) {

		auth_data[i] = &UAS_data.Auth[i];

		auth_data[i]->DataPage = i;
		auth_data[i]->AuthType = ODID_AUTH_NONE; // 0
	}

	basicID_data->IDType              = ODID_IDTYPE_NONE; // 0
	basicID_data->UAType              = ODID_UATYPE_NONE; // 0

	odid_initLocationData(location_data);

	location_data->Status             = ODID_STATUS_UNDECLARED; // 0
	location_data->SpeedVertical      = INV_SPEED_V;
	location_data->HeightType         = ODID_HEIGHT_REF_OVER_TAKEOFF;
	location_data->HorizAccuracy      = ODID_HOR_ACC_10_METER;
	location_data->VertAccuracy       = ODID_VER_ACC_10_METER;
	location_data->BaroAccuracy       = ODID_VER_ACC_10_METER;
	location_data->SpeedAccuracy      = ODID_SPEED_ACC_10_METERS_PER_SECOND;
	location_data->TSAccuracy         = ODID_TIME_ACC_1_0_SECOND;

	selfID_data->DescType             = ODID_DESC_TYPE_TEXT;
	strcpy(selfID_data->Desc, "Recreational");

	odid_initSystemData(system_data);

	system_data->OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
	system_data->ClassificationType   = ODID_CLASSIFICATION_TYPE_EU;
	system_data->AreaCount            = 1;
	system_data->AreaRadius           = 500;
	system_data->AreaCeiling          =
	system_data->AreaFloor            = -1000.0;
	system_data->CategoryEU           = ODID_CATEGORY_EU_SPECIFIC;
	system_data->ClassEU              = ODID_CLASS_EU_UNDECLARED;
	system_data->OperatorAltitudeGeo  = -1000.0;

	operatorID_data->OperatorIdType   = ODID_OPERATOR_ID; 

}

void ID_OpenDrone_param_init(struct UTM_parameters *parameters)
{
	int status;
	char text[128];

#if ID_OD_WIFI

	int i;
	int8_t wifi_power;
	wifi_config_t wifi_config;
#endif  //ID_OD_WIFI

	status = 0;
	text[0] = 0;
	text[63] = 0;

	//operator

	UAS_operator = parameters->UAS_operator;

	strncpy(operatorID_data->OperatorId, parameters->UAS_operator, ODID_ID_SIZE);
	operatorID_data->OperatorId[sizeof(operatorID_data->OperatorId) - 1] = 0;
	
	//basic

	basicID_data->UAType = (ODID_uatype_t)parameters->UA_type;
	basicID_data->IDType = (ODID_idtype_t)parameters->ID_type;

	switch (basicID_data->IDType)
	{
	case ODID_IDTYPE_SERIAL_NUMBER:
		strncpy(basicID_data->UASID, parameters->UAV_id, ODID_ID_SIZE);
		break;
	case ODID_IDTYPE_CAA_REGISTRATION_ID:
		strncpy(basicID_data->UASID, parameters->UAS_operator, ODID_ID_SIZE);
		break;
	default:
		break;
	}

	basicID_data->UASID[sizeof(basicID_data->UASID) - 1] = 0;

	//system

	if (parameters->region < 2)
	{
		system_data->ClassificationType = (ODID_classification_type_t)parameters->region;
	}
	if (parameters->EU_category < 4)
	{
		system_data->CategoryEU = (ODID_category_EU_t)parameters->EU_category;
	}
	if (parameters->EU_class < 8)
	{
		system_data->ClassEU = (ODID_class_EU_t)parameters->EU_class;
	}

	encodeBasicIDMessage(&basicID_enc, basicID_data);
	encodeLocationMessage(&location_enc, location_data);
	encodeAuthMessage(&auth_enc, auth_data[0]);
	encodeSelfIDMessage(&selfID_enc, selfID_data);
	encodeSystemMessage(&system_enc, system_data);
	encodeOperatorIDMessage(&operatorID_enc, operatorID_data);

#if ID_OD_WIFI

	if (UAS_operator[0])
	{
		strncpy(ssid, UAS_operator, i = sizeof(ssid));
		ssid[i - 1] = 0;
	}

	ssid_length = strlen(ssid);

	//  new WiFi softAP
	//wifi_mode_t currentMode = getMode();
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_ap();
	
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	
	strncpy(&wifi_config.ap.ssid, ssid, i = sizeof(ssid));
	wifi_config.ap.ssid_len = ssid_length;
	wifi_config.ap.password[0] = 0;
	wifi_config.ap.channel = wifi_channel;
	wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	wifi_config.ap.ssid_hidden = 1;
	wifi_config.ap.beacon_interval = 100;
	wifi_config.ap.max_connection = 4;
	wifi_config.ap.ftm_responder = false;
	
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
	
	ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
	
	ESP_ERROR_CHECK(esp_wifi_start());
	
	ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&wifi_power));
	
	esp_read_mac(WiFi_mac_addr, ESP_MAC_WIFI_STA);
	
#if ID_OD_WIFI_BEACON & !USE_BEACON_FUNC
	
	struct __attribute__((__packed__)) beacon_header {

		uint8_t control[2];          //  0-1:  frame control  
		uint8_t duration[2];         //  2-3:  duration
		uint8_t dest_addr[6];        //  4-9:  destination
		uint8_t src_addr[6];         // 10-15: source  
		uint8_t bssid[6];            // 16-21: base station
		uint8_t seq[2];              // 22-23: sequence
		uint8_t timestamp[8];        // 24-31: 
		uint8_t interval[2];         //
		uint8_t capability[2];
	} *header;

	header                  = (struct beacon_header *) beacon_frame;
	beacon_timestamp        = header->timestamp;

	header->control[0]      = 0x80;
	header->interval[0]     = 0xb8;
	header->interval[1]     = 0x0b;
	header->capability[0]   = 0x21; // ESS | Short preamble
	header->capability[1]   = 0x04; // Short slot time

	for (i = 0; i < 6; ++i) {

		header->dest_addr[i] = 0xff;
		header->src_addr[i]  = 
		header->bssid[i]     = WiFi_mac_addr[i];
	}
  
	beacon_offset = sizeof(struct beacon_header);

	beacon_frame[beacon_offset++] = 0;
	beacon_frame[beacon_offset++] = ssid_length;

	for (i = 0; (i < 32)&&(ssid[i]); ++i) {

		beacon_frame[beacon_offset++] = ssid[i];
	}
	
	// Supported rates
#if 1
  beacon_frame[beacon_offset++] = 0x01; // This is what ODID 1.0 does.
	beacon_frame[beacon_offset++] = 0x01;
	beacon_frame[beacon_offset++] = 0x8c; // 11b, 6(B) Mbit/sec
#elif 0
	beacon_frame[beacon_offset++] = 0x01; // This is what the ESP32's beacon frames do. Jams GPS?
	beacon_frame[beacon_offset++] = 0x08;
	beacon_frame[beacon_offset++] = 0x8b; //  5.5
	beacon_frame[beacon_offset++] = 0x96; // 11
	beacon_frame[beacon_offset++] = 0x82; //  1
	beacon_frame[beacon_offset++] = 0x84; //  2
	beacon_frame[beacon_offset++] = 0x0c; //  6, note not 0x8c
	beacon_frame[beacon_offset++] = 0x18; // 12 
	beacon_frame[beacon_offset++] = 0x30; // 24
	beacon_frame[beacon_offset++] = 0x60; // 48
#endif
	
	// DS
	beacon_frame[beacon_offset++] = 0x03;
	beacon_frame[beacon_offset++] = 0x01;
	beacon_frame[beacon_offset++] = wifi_channel;
  
	// payload
	beacon_payload      = &beacon_frame[beacon_offset];
	beacon_offset      += 7;

	*beacon_payload++   = 0xdd;
	beacon_length       = beacon_payload++;

	*beacon_payload++   = 0xfa;
	*beacon_payload++   = 0x0b;
	*beacon_payload++   = 0xbc;

	*beacon_payload++   = 0x0d;
	beacon_counter      = beacon_payload++;

	beacon_max_packed   = BEACON_FRAME_SIZE - beacon_offset - 2;

	if (beacon_max_packed > (ODID_PACK_MAX_MESSAGES * ODID_MESSAGE_SIZE)) {

		beacon_max_packed = (ODID_PACK_MAX_MESSAGES * ODID_MESSAGE_SIZE);
	}
	
#endif  // ID_OD_WIFI_BEACON & !USE_BEACON_FUNC
	

#endif  //ID_OD_WIFI

#if ID_OD_BT
	
#endif  //ID_OD_BT




}

void ID_OpenDrone_set_auth(char *auth)
{
	ID_OpenDrone_set_auth_set((uint8_t *)auth, strlen(auth), 0x0a);
}

void ID_OpenDrone_set_auth_set(uint8_t *auth, short int len, uint8_t type)
{
	int	i, j;
	//char text[160]; // for debug
	uint8_t check[32];
	uint32_t secs;
	
	auth_page_count = 1;
	secs = 0;
	
	if (len > MAX_AUTH_LENGTH)
	{
		
		len = MAX_AUTH_LENGTH;
		auth[len] = 0;
	}
	
	auth_data[0]->AuthType = (ODID_authtype_t)type;
	
	for (i = 0;(i < 17)&&(auth[i]);++i)
	{
		check[i] = 
		auth_data[0]->AuthData[i] = auth[i];
	}
	
	check[i] = 
	auth_data[0]->AuthData[i] = 0;
	
	if (len > 16)
	{
		for (auth_page_count = 1; (auth_page_count < ODID_AUTH_MAX_PAGES)&&(i < len); ++auth_page_count)
		{
			auth_data[auth_page_count]->AuthType = (ODID_authtype_t)type;
			
			for (j = 0; (j < 23)&&(i < len); ++i, ++j)
			{
				check[j] = 
				auth_data[auth_page_count]->AuthData[j] = auth[i];
			}
			
			if (j < 23)
			{
				auth_data[auth_page_count]->AuthData[j] = 0;
			}
      
			check[j] = 0;
			
		}
		len = i;
	}
	auth_data[0]->LastPageIndex = (auth_page_count) ? auth_page_count - 1 : 0;
	auth_data[0]->Length        = len;
	auth_data[0]->Timestamp     = (uint32_t)(secs - ID_OD_AUTH_DATUM);
}

int ID_OpenDrone_transmit(struct UTM_data *utm_data)
{
	int					i, status,
						valid_data, wifi_tx_flag_1, wifi_tx_flag_2;
	char				text[128];
	static uint32_t			msecs = 0;
	static uint32_t			secs = 0;
	static int			phase = 0;
	static uint32_t		last_msecs = 0;
	
	text[0] = 0;
	msecs += 100;
	if (msecs > 900)
	{
		secs += 1;	
	}
	
	
	if ((!system_data->OperatorLatitude)&&(utm_data->base_valid)) 
	{
		system_data->OperatorLatitude    = utm_data->base_latitude;
		system_data->OperatorLongitude   = utm_data->base_longitude;
		system_data->OperatorAltitudeGeo = utm_data->base_alt_m;

		system_data->Timestamp           = (uint32_t)(secs - ID_OD_AUTH_DATUM);

		encodeSystemMessage(&system_enc, system_data);
	}
	
	valid_data               =
	wifi_tx_flag_1           =
	wifi_tx_flag_2           = 0;

	UAS_data.BasicIDValid[0] =
	UAS_data.LocationValid   =
	UAS_data.SelfIDValid     =
	UAS_data.SystemValid     =
	UAS_data.OperatorIDValid = 0;
	
	for (i = 0; i < ODID_AUTH_MAX_PAGES; ++i) 
	{
		UAS_data.AuthValid[i] = 0;
	}
	/*
	if ((msecs - last_msecs) > 74)
	{
		last_msecs = (last_msecs) ? last_msecs + 75 : msecs;

		switch (++phase) {

		case  4: case  8: case 12: // Every 300 ms.
		case 16: case 20: case 24:
		case 28: case 32: case 36:

			wifi_tx_flag_1 = 1;

			if (utm_data->satellites >= SATS_LEVEL_2) {

				location_data->Direction       = (float) utm_data->heading;
				location_data->SpeedHorizontal = 0.514444 * (float) utm_data->speed_kn;
				location_data->SpeedVertical   = INV_SPEED_V;
				location_data->Latitude        = utm_data->latitude_d;
				location_data->Longitude       = utm_data->longitude_d;
				location_data->Height          = utm_data->alt_agl_m;
				location_data->AltitudeGeo     = utm_data->alt_msl_m;
    
				location_data->TimeStamp       = (float)((utm_data->minutes * 60) + utm_data->seconds) +
				                                 0.01 * (float) utm_data->csecs;

				if ((status = encodeLocationMessage(&location_enc, location_data)) == ODID_SUCCESS) {

					valid_data = UAS_data.LocationValid = 1;

					ID_OpenDrone_transmit_ble((uint8_t *) &location_enc, sizeof(location_enc));

				}
			}

			break;

		case  6:

			if (basicID_data->IDType) {
        
				valid_data = UAS_data.BasicIDValid[0] = 1;
				ID_OpenDrone_transmit_ble((uint8_t *) &basicID_enc, sizeof(basicID_enc));
			}
      
			break;

		case 14:

			wifi_tx_flag_2 = valid_data =
			UAS_data.SelfIDValid        = 1;
			ID_OpenDrone_transmit_ble((uint8_t *) &selfID_enc, sizeof(selfID_enc));
			break;

		case 22:

			valid_data = UAS_data.SystemValid = 1;
#if 1
			if (secs > ID_OD_AUTH_DATUM) {

				system_data->Timestamp = (uint32_t)(secs - ID_OD_AUTH_DATUM);
				encodeSystemMessage(&system_enc, system_data);
			}
#endif
			ID_OpenDrone_transmit_ble((uint8_t *) &system_enc, sizeof(system_enc));
			break;

		case 30:

			valid_data = UAS_data.OperatorIDValid = 1;
			ID_OpenDrone_transmit_ble((uint8_t *) &operatorID_enc, sizeof(operatorID_enc));
			break;

		case 38:

			if (auth_page_count) {

				encodeAuthMessage(&auth_enc, auth_data[auth_page]);
				valid_data = UAS_data.AuthValid[auth_page] = 1;

				ID_OpenDrone_transmit_ble((uint8_t *) &auth_enc, sizeof(auth_enc));

				if (++auth_page >= auth_page_count) {

					auth_page = 0;
				}
			}

			break;

		default:

			if (phase > 39) {

				phase = 0;
			}

			break;
		}
	}
	*/
	
#if ID_OD_WIFI

#if 0
	if (valid_data)
	{
		status = ID_OpenDrone_transmit_wifi(utm_data);
	}
	
#else
	
	
	sPosition[1].heading = sPosition[0].heading;
	sPosition[1].latitude = sPosition[0].latitude;
	sPosition[1].longitude = sPosition[0].longitude;
	sPosition[1].x = sPosition[0].x;
	sPosition[1].y = sPosition[0].y;
	sPosition[1].speed = sPosition[0].speed;
	
	sPosition[0] = circle_path(id, 36.741679, 127.119694, 100);

	sPosition[0].speed = sqrt((sPosition[0].x - sPosition[1].x)*(sPosition[0].x - sPosition[1].x)
								+ (sPosition[0].y - sPosition[1].y)*(sPosition[0].y - sPosition[1].y));
	sPosition[0].heading = atan((sPosition[0].x - sPosition[1].x) / (sPosition[0].y - sPosition[1].y)) * 180 / 3.14;
	
	//printf("%lf %lf %lf %lf\r\n", sPosition[0].latitude, sPosition[0].longitude, sPosition[0].heading, sPosition[0].speed);
	// Pack the WiFi data.
	// One group every 300ms and another every 3000ms.
	wifi_tx_flag_1 = 1;
	
	if (wifi_tx_flag_1)
	{
		UAS_data.SystemValid = 1;
		// encode system message 
		{
			system_data->OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_FIXED; // 0=TakeOff, 1=Live GNSS, 2=Fixed Location
			system_data->ClassificationType = ODID_CLASSIFICATION_TYPE_EU;
			system_data->OperatorLatitude = g_OperatorLatitude;
			system_data->OperatorLongitude = g_OperatorLongitude;
			system_data->AreaCount = 1;
			system_data->AreaRadius = 0;
			system_data->AreaCeiling = 0;
			system_data->AreaFloor = 0;
			system_data->CategoryEU = ODID_CATEGORY_EU_UNDECLARED;
			system_data->ClassEU = ODID_CLASS_EU_UNDECLARED;
			system_data->Timestamp = 0;
		}
		
		UAS_data.LocationValid = 1;
		// encode location message
		{
			//location_data->Direction       = 60.0;//(float) utm_data->heading;
			location_data->Direction       = sPosition[0].heading; //(float) utm_data->heading;
			//location_data->SpeedHorizontal = 13;//0.514444 * (float) utm_data->speed_kn;
			location_data->SpeedHorizontal = sPosition[0].speed; //0.514444 * (float) utm_data->speed_kn;
			location_data->SpeedVertical   = INV_SPEED_V;
			//location_data->Latitude        = 36.741679;//utm_data->latitude_d;
			//location_data->Longitude       = 127.119694;//utm_data->longitude_d;
			location_data->Latitude        = sPosition[0].latitude; //utm_data->latitude_d;
			location_data->Longitude       = sPosition[0].longitude; //utm_data->longitude_d;
			location_data->Height          = 27;//utm_data->alt_agl_m;
			location_data->AltitudeGeo     = 34;//utm_data->alt_msl_m;
    
			location_data->TimeStamp       = (float)((utm_data->minutes * 60) + utm_data->seconds) +
			                                 0.01 * (float) utm_data->csecs;
		}
		
		UAS_data.SelfIDValid = 1;
		// encode selfid message
		{
			selfID_data->DescType = 0;
			//selfID_data->Desc = NULL;
			if (id == 0)
			{
			//	strcpy(selfID_data->Desc, "TEST_MODULE_NO01");
			}
			else if (id == 1)
			{
				strcpy(selfID_data->Desc, "TEST_MODULE_NO02");
			}
			else if (id == 2)
			{
				strcpy(selfID_data->Desc, "TEST_MODULE_NO03");
			}
			strcpy(selfID_data->Desc, "KATECH_TEST_MODULE");
			
		}
		
		//if (UAS_data.BasicID[0].UASID[0])
		{
			UAS_data.BasicIDValid[0] = 1;
			//encode basicid message 
			{
				basicID_data->UAType = 2;
				basicID_data->IDType = 3;
				
				if (id == 0)
				{
				//	strcpy(basicID_data->UASID, "2302221");
				}
				else if (id == 1)
				{
					strcpy(basicID_data->UASID, "2302222");
				}
				else if (id == 2)
				{
					strcpy(basicID_data->UASID, "2302223");
				}
				strcpy(basicID_data->UASID, "1234555");
			}
		}
		
		//if (UAS_data.OperatorID.OperatorId[0])
		{
			UAS_data.OperatorIDValid = 1;
			// encode operatorid message
			{
				operatorID_data->OperatorIdType = ODID_OPERATOR_ID;
				
				if (id == 0)
				{
				//	strcpy(operatorID_data->OperatorId, "ASSETTA_USER_01");
				}
				else if (id == 1)
				{
					strcpy(operatorID_data->OperatorId, "ASSETTA_USER_02");
				}
				else if (id == 2)
				{
					strcpy(operatorID_data->OperatorId, "ASSETTA_USER_03");
				}
				strcpy(operatorID_data->OperatorId, "KATECH_USER");
			}
		}
//		for (i = 0; (i < auth_page_count)&&(i < ODID_AUTH_MAX_PAGES); ++i)
//		{
//			UAS_data.AuthValid[i] = 1;
//		}
		printf("%lf %lf\r\n", location_data->Latitude, location_data->Longitude);
		status = ID_OpenDrone_transmit_wifi(utm_data);
		
//		if (id >= 2)
//		{
//			id = 0;
//		}
//		else 
//		{
//			id++;
//		}
		
	}
/*	else if(wifi_tx_flag_2)
	{
		for (i = 0; (i < auth_page_count)&&(i < ODID_AUTH_MAX_PAGES); ++i)
		{
			UAS_data.AuthValid[i] = 1;
		}
		
		status = ID_OpenDrone_transmit_wifi(utm_data);
	}
*/
	
#endif  //0 Pack data
	
#endif  //ID_OD_WIFI
	
	return 0;
}

int ID_OpenDrone_transmit_wifi(struct UTM_data *utm_data)
{
#if ID_OD_WIFI
	int			length;
	esp_err_t	wifi_status;
	
#if ID_OD_WIFI_NAN
	
	char				text[128];
	uint8_t				buffer[1024];
	static uint8_t		send_counter = 0;
	
	if ((length = odid_wifi_build_nan_sync_beacon_frame((char *)WiFi_mac_addr, buffer, sizeof(buffer))) > 0)
	{
		wifi_status = esp_wifi_80211_tx(WIFI_IF_AP, buffer, length, true);
	}
	
	if ((length = odid_wifi_build_message_pack_nan_action_frame(&UAS_data, (char *)WiFi_mac_addr, ++send_counter, buffer, sizeof(buffer))) > 0)
	{
		//printf("%lf\r\n", UAS_data.Location.Latitude);
		wifi_status = esp_wifi_80211_tx(WIFI_IF_AP, buffer, length, true);
	}
	
#endif  //NAN
	
#if ID_OD_WIFI_BEACON
	
#if USE_BEACON_FUNC
	
	if ((length = odid_wifi_build_message_pack_beacon_frame(&UAS_data,
															(char *) WiFi_mac_addr,
															ssid,
															ssid_length,
															3000,
															++beacon_counter,
															beacon_frame,
															BEACON_FRAME_SIZE)) > 0) {

		wifi_status = esp_wifi_80211_tx(WIFI_IF_AP, beacon_frame, length, true);
	}
	
#else

	int      i, len2 = 0;
	static uint64_t usecs = 0;

	++*beacon_counter;

	usecs += 100; //micros();

	for (i = 0; i < 8; ++i)
	{

		beacon_timestamp[i] = (usecs >> (i * 8)) & 0xff;
	}

	if ((length = odid_message_build_pack(&UAS_data, beacon_payload, beacon_max_packed)) > 0)
	{
		*beacon_length = length + 5;
    
		wifi_status = esp_wifi_80211_tx(WIFI_IF_AP, beacon_frame, len2 = beacon_offset + length, true);
	}

#endif	//FUNC
	
#endif  //BEACON
	
#endif  // WIFI
	
	return 0;
}

int ID_OpenDrone_transmit_ble(uint8_t *odid_msg, int length)
{
#if ID_OD_BT
	
	int			i, j, k, len;
	uint8_t		*a;
	esp_err_t	status;
	
	i = j = k = len = 0;
	a = ble_message;
	
	memset(ble_message, 0, sizeof(ble_message));
	
	if (advertising)
	{
		status = esp_ble_gap_stop_advertising();
	}
	
	ble_message[j++] = 0x1e;
	ble_message[j++] = 0x16;
	ble_message[j++] = 0xfa; // ASTM
	ble_message[j++] = 0xff; //
	ble_message[j++] = 0x0d;
	
#if 0
	ble_message[j++] = ++counter;
#else
	ble_message[j++] = ++msg_counter[odid_msg[0] >> 4];
#endif
	
	for (i = 0; (i < length)&&(j < sizeof(ble_message)); ++i, ++j) 
	{
		ble_message[j] = odid_msg[i];
	}

	status = esp_ble_gap_config_adv_data_raw(ble_message, len = j); 
	status = esp_ble_gap_start_advertising(&advParams);

	advertising = 1;
	
#endif //BT
	return 0;
}

void ID_OpenDrone_utm_message_pack(ODID_UAS_Data *outdata, struct UTM_data *indata)
{
	// BasicID
	basicID_data->UAType = 2;
	basicID_data->IDType = 3;
	strcpy(basicID_data->UASID, "3406820");
	

	
	// Location
	location_data->Direction       = (float) indata->heading;
	location_data->SpeedHorizontal = 0.514444 * (float) indata->speed_kn;
	location_data->SpeedVertical   = INV_SPEED_V;
	location_data->Latitude        = indata->latitude_d;
	location_data->Longitude       = indata->longitude_d;
	location_data->Height          = indata->alt_agl_m;
	location_data->AltitudeGeo     = indata->alt_msl_m;
    
	location_data->TimeStamp       = (float)((indata->minutes * 60) + indata->seconds) +
	                                 0.01 * (float) indata->csecs;
	
	// SelfID
//	selfID_data->DescType = 0;
//	strcpy(selfID_data->Desc, "SelfID");
	//selfID_data->Desc = NULL;
	
	// System
	system_data->OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_FIXED; // 0=TakeOff, 1=Live GNSS, 2=Fixed Location
	system_data->ClassificationType = ODID_CLASSIFICATION_TYPE_EU;
	system_data->OperatorLatitude = 36.740751;
	system_data->OperatorLongitude = 127.119139;
	system_data->AreaCount = 1;
	system_data->AreaRadius = 0;
	system_data->AreaCeiling = 0;
	system_data->AreaFloor = 0;
	system_data->CategoryEU = ODID_CATEGORY_EU_UNDECLARED;
	system_data->ClassEU = ODID_CLASS_EU_UNDECLARED;
	system_data->Timestamp = 0;
	
	//operatorID_data;
	operatorID_data->OperatorIdType = ODID_OPERATOR_ID;
	strcpy(operatorID_data->OperatorId, "OPERATORID");

}

const double xxa = 6378137.0;
const double xxf = 0.0033528106647474805;
const double xxfe = 500000.0;
const double xxok = 0.9996;
const double xxrecf = 298.25722356300003;
const double xxb = 6356752.3142451793;
const double xxes = 0.0066943799901413156;
const double xxebs = 0.0067394967422764341;
const double xxtn = 0.0016792203863837205;
const double xxap = 6367449.1458234154;
const double xxbp = 16038.508662976265;
const double xxcp = 16.832613263150265;
const double xxdp = 0.021984404134262194;
const double xxep = 3.1148371055723339e-005;


void  GPS_gp2utm(double sphi, double slam, int *izone, double *y, double *x)
{
	double t = 0, t2 = 0, t4 = 0, t6 = 0;
	double c = 0, c2 = 0, c3 = 0, c5 = 0, c7 = 0;
	double eta = 0, eta2 = 0, eta3 = 0, eta4 = 0;
	double dlam2 = 0, dlam3 = 0, dlam4 = 0, dlam5 = 0, dlam6 = 0, dlam7 = 0, dlam8 = 0;
	double olam = 0, dlam = 0, s = 0, sn = 0, tmd = 0;
	double xt1 = 0, xt2 = 0, xt3 = 0, xt4 = 0, xt5 = 0, xt6 = 0, xt7 = 0, xt8 = 0, xt9 = 0, nfn = 0;
	double degrad = atan(1.) / 45.;
	double ftemp = 0;
	double fS2 = 0, fS4 = 0, fS6 = 0, fS8 = 0;

	sphi *= degrad;
	slam *= degrad;
	if ((degrad > 0) || (degrad < 0)) 
	{
		*izone = 31 + (int)(slam / degrad / 6.0);
	}

	olam = (((double)*izone) * 6 - 183)*degrad;
	dlam = slam - olam;
	dlam2 = dlam * dlam;
	dlam3 = dlam2 * dlam;
	dlam4 = dlam3 * dlam;
	dlam5 = dlam4 * dlam;
	dlam6 = dlam5 * dlam;
	dlam7 = dlam6 * dlam;
	dlam8 = dlam7 * dlam;

	s = sin(sphi);
	c = cos(sphi);
	c2 = c * c;
	c3 = c2 * c;
	c5 = c3 * c2;
	c7 = c5 * c2;

	if ((c > 1.0E-19) || (c < -1.0E-19)) 
	{
		t = s / c;
	}
	else
	{
		c = 1.0E-19;
		t = s / c;
	}
	t2 = t * t;
	t4 = t2 * t2;
	t6 = t4 * t2;

	eta = xxebs * c2;
	eta2 = eta * eta;
	eta3 = eta2 * eta;
	eta4 = eta2 * eta2;

	ftemp = sqrt(1.0 - xxes*s*s);
	if ((ftemp > 0) || (ftemp < 0))
	{
		sn = xxa / ftemp;
	}
	
	fS2 = sin(2*sphi);
	fS4 = sin(4*sphi);
	fS6 = sin(6*sphi);
	fS8 = sin(8*sphi);
	tmd = xxap*sphi - xxbp*fS2 + xxcp*fS4 - xxdp*fS6 + xxep*fS8;

	xt1 = tmd*xxok;
	xt2 = sn*s*c*xxok / 2.0;
	xt3 = sn*s*c3*xxok*(5.0 - t2 + 9.0*eta + 4.0*eta2) / 24.0;
	xt4 = sn*s*c5*xxok*(61.0 - 58.0*t6 + 270.0*eta - 330.0*t2*eta
		 + 445.0*eta2 + 324.*eta3 - 680*eta2*t2 + 88.*eta4
		 - 600.0*t2*eta3 - 192.0*t2*eta4) / 720.0;
	xt5 = sn*s*c7*xxok*(1385.0 - 3111.0*t2 + 543.0*t4 - t6) / 40320.0;

	nfn = 0.;
	if (sphi < 0.0) 
	{
		nfn = 10000000.0;
	}

	*y = nfn + xt1 + dlam2*xt2 + dlam4*xt3 + dlam6*xt4 + dlam8*xt5;

	xt6 = sn*c*xxok;
	xt7 = sn*c3*xxok*(1.0 - t2 + eta) / 6.0;
	xt8 = sn*c5*xxok*(5.0 - 18.0*t6 + 14.0*eta - 58.0*t2*eta
		 + 13.0*eta2 + 4.0*eta3 - 64.0*eta2*t2 - 24.0*eta4) / 120.0;
	xt9 = sn*c7*xxok*(61.0 - 479.0*t2 + 179.0*t4 - t6) / 5040.0;

	*x = xxfe + dlam*xt6 + dlam3*xt7 + dlam5*xt8 + dlam7*xt9;
}

/*==============================================================================================

	INPUT ARGUEMENT :
	
		izone	  : UTM zone number 
		y         : UTM northing (meters)
		x 		  : UTM easting  (meters)    


	OUTPUT ARGUEMENTS :

		sphi      : Latitude  (deg)
		slam 	  : Longitude (deg)

==============================================================================================*/
void GPS_utm2gp(double *sphi, double *slam, int izone, double y, double x)
{
	double degrad = 0;
	double olam = 0, dlam = 0, s = 0, c = 0, t = 0, eta = 0, sn = 0, tmd = 0, sr = 0, ftphi = 0, de = 0;
	double t10 = 0, t11 = 0, t12 = 0, t13 = 0, t14 = 0, t15 = 0, t16 = 0, t17 = 0, nfn = 0;
	int i = 0;
	double ftemp = 0;
	double fS1 = 0, fS2 = 0, fS4 = 0, fS6 = 0, fS8 = 0;
	double fSqrt = 0;

	degrad = atan(1.) / 45.;
	nfn = 0.0;
	if (y < 0.0) {
		nfn = 10000000.;
		y = fabs(y);
	}

	if ((xxok > 0) || (xxok < 0))
	{
		tmd = (y - nfn) / xxok;
	}
	fS1 = sin(*sphi);
	fSqrt = sqrt(1. - xxes*fS1*fS1);
	ftemp = fSqrt*fSqrt*fSqrt;
	if ((ftemp > 0) || (ftemp < 0))
	{
		sr = xxa*(1. - xxes) / ftemp;
	}
	if ((sr < 1.0E-19) && (sr > -1.0E-19))
	{
		sr = 1.0E-19;
	}
	if ((sr > 0) || (sr < 0))
	{
		ftphi = tmd / sr;
	}

	for (i = 0; i < 5; i++) {
		fS2 = sin(2*ftphi);
		fS4 = sin(4*ftphi);
		fS6 = sin(6*ftphi);
		fS8 = sin(8*ftphi);
		t10 = xxap*ftphi - xxbp*fS2 + xxcp*fS4 - xxdp*fS6 + xxep*fS8;
		fS1 = sin(ftphi);
		fSqrt = sqrt(1. - xxes*fS1*fS1);
		ftemp = fSqrt*fSqrt*fSqrt;
		if ((ftemp > 0) || (ftemp < 0))
		{
			sr = xxa*(1. - xxes) / ftemp;
		}
		if ((sr < 1.0E-19) && (sr > -1.0E-19))
		{
			sr = 1.0E-19;
		}
		if ((sr > 0) || (sr < 0))
		{
			ftphi += (tmd - t10) / sr;
		}
	}

	fS1 = sin(ftphi);
	fSqrt = sqrt(1. - xxes*fS1*fS1);
	ftemp = fSqrt*fSqrt*fSqrt;
	if ((ftemp > 0) || (ftemp < 0))
	{
		sr = xxa*(1. - xxes) / ftemp;
	}
	fS1 = sin(ftphi);
	ftemp = sqrt(1. - xxes*fS1*fS1);
	if ((ftemp > 0) || (ftemp < 0))
	{
		sn = xxa / ftemp;
	}

	s = sin(ftphi);
	c = cos(ftphi);
	if ((c < 1.0E-19) && (c > -1.0E-19))
	{
		c = 1.0E-19;
	}
	if ((c > 0) || (c < 0))
	{
		t = s / c;
	}
	eta = xxebs*c*c;

	de = x - xxfe;

	ftemp = (2.*sr*sn*xxok*xxok);
	if ((ftemp > 0) || (ftemp < 0))
	{
		t10 = t / ftemp;
	}
	ftemp = (24.*sr*sn*sn*sn*xxok*xxok*xxok*xxok);
	if ((ftemp > 0) || (ftemp < 0))
	{
		t11 = t*(5. + 3.*t*t + eta - 4.*eta*eta - 9.*t*t*eta) / ftemp;
	}
	ftemp = (720.*sr*sn*sn*sn*sn*sn*xxok*xxok*xxok*xxok*xxok*xxok);
	if ((ftemp > 0) || (ftemp < 0))
	{
		t12 = t*(61. + 90.*t*t + 46.*eta + 45.*t*t*t*t - 252.*t*t*eta - 3.*eta*eta + 100.*eta*eta*eta - 66.*t*t*eta*eta - 90.*t*t*t*t*eta
		  + 88.*eta*eta*eta*eta + 225.*t*t*t*t*eta*eta + 84.*t*t*eta*eta*eta - 192.*t*t*eta*eta*eta*eta) / ftemp;
	}
	ftemp = (40320.*sr*sn*sn*sn*sn*sn*sn*sn*xxok*xxok*xxok*xxok*xxok*xxok*xxok*xxok);
	if ((ftemp > 0) || (ftemp < 0))
	{
		t13 = t*(1385. + 3633.*t*t + 4095.*t*t*t*t + 1575.*t*t*t*t*t*t) / ftemp;
	}

	if (degrad > 0)
	{
		*sphi = (ftphi - de*de*t10 + de*de*de*de*t11 - de*de*de*de*de*de*t12 + de*de*de*de*de*de*de*de*t13) / degrad;
	}

	ftemp = (sn*c*xxok);
	if ((ftemp > 0) || (ftemp < 0))
	{
		t14 = 1. / ftemp;
	}
	ftemp = (6.*sn*sn*sn*c*xxok*xxok*xxok);
	if ((ftemp > 0) || (ftemp < 0))
	{
		t15 = (1. + 2.*t*t + eta) / ftemp;
	}
	ftemp = (120.*sn*sn*sn*sn*sn*c*xxok*xxok*xxok*xxok*xxok);
	if ((ftemp > 0) || (ftemp < 0))
	{
		t16 = (5. + 6.*eta + 28.*t*t - 3.*eta*eta + 8.*t*t*eta + 24.*t*t*t*t - 4.*eta*eta*eta + 4.*t*t*eta*eta + 24.*t*t*eta*eta*eta) / ftemp;
	}
	ftemp = (5040.*sn*sn*sn*sn*sn*sn*sn*c*xxok*xxok*xxok*xxok*xxok*xxok*xxok);
	if ((ftemp > 0) || (ftemp < 0))
	{
		t17 = (61. + 662.*t*t + 1320.*t*t*t*t + 720.*t*t*t*t*t*t) / ftemp;
	}

	dlam = de*t14 - de*de*de*t15 + de*de*de*de*de*t16 - de*de*de*de*de*de*de*t17;

	olam = ((double)izone * 6.0 - 183)*degrad;
	if ((degrad > 0) || (degrad < 0))
	{
		*slam = (olam + dlam) / degrad;
	}
}


struct struct_Position circle_path(unsigned char id, double lat, double lon, double radius)
{
	int izone = 0;
	double x = 0, y = 0;
	double c_x = 0, c_y = 0;
	double c_lon = 0, c_lat = 0;
	static unsigned int second = 0;
	struct struct_Position return_position;
	
	static double theta = 0, w = 0.07;	// 'w' is rad/s
	static double heading = 0;
	
	memset(&return_position, 0, sizeof(return_position));
	/*	INPUT ARGUEMENT :
		sphi      : Latitude  (deg)
		slam 	  : Longitude (deg)
	OUTPUT ARGUEMENTS :
		izone	  : UTM zone number (see INPUT ARGUEMENT)
		y         : UTM northing (meters)
		x 		  : UTM easting  (meters)
	*/
	//void  GPS_gp2utm(double sphi, double slam, int *izone, double *y, double *x)
	GPS_gp2utm(lat, lon, &izone, &y, &x);
	theta = w * second++;
	
	c_x = radius * cos(theta * 3.14 / 180) + x;
	c_y = radius * sin(theta * 3.14 / 180) + y;
	
	/*
	INPUT ARGUEMENT :
	
		izone	  : UTM zone number 
		y         : UTM northing (meters)
		x 		  : UTM easting  (meters)    


	OUTPUT ARGUEMENTS :

		sphi      : Latitude  (deg)
		slam 	  : Longitude (deg)
	*/
	//GPS_utm2gp(double *sphi,double *slam, int izone, double y, double x)
	//GPS_utm2gp(&c_lon, &c_lat, izone, c_y, c_x);
	GPS_utm2gp(&c_lat, &c_lon, izone, c_y, c_x);

//	printf("%lf\r\n", lat);
//	printf("%lf\r\n", lon);
	
//	printf("%lf\r\n", c_lat);
//	printf("%lf\r\n", c_lon);

	
	heading = 0;//atan(sqrt((c_x - x)*(c_x - x) + (c_y - y)*(c_y - y))) * 180 / 3.14;
	
	//printf("%lf\r\n", heading);
	
	/*	double latitude;
		double longitude;
		double x;
		double y;
		double heading;
		*/
	return_position.latitude = c_lat;
	return_position.longitude = c_lon;
	return_position.x = c_x;
	return_position.y = c_y;
	return_position.heading = heading;
	return_position.speed = 0;//sqrt((c_x - x)*(c_x - x) + (c_y - y)*(c_y - y)); // m/s
	
	return return_position;
}