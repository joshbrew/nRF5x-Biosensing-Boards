#include <zephyr.h>

#include <bluetooth/gatt.h>
#include <bluetooth/uuid.h>
#include "ble_service.hpp"
#include <functional>

#include <logging/log.h>

#include <sys/atomic.h>

#include "ble_gatt.hpp"
#include "qmc5883l.hpp"

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <sys/byteorder.h>

namespace Bluetooth::Gatt
{

LOG_MODULE_REGISTER(BleGatt, LOG_LEVEL_INF);

constexpr static size_t controlHeaderSize = 3;
constexpr static size_t maxHandlers = 256;
constexpr static size_t IBEACON_MAX_LENGTH = 21;
constexpr static size_t IBEACON_TYPE = 0x02;

/**
 * @brief When BLE Performs GATT write it might split transfer into several chunks, and only first byte contains 
 *        function ID, so this funtion should be stored
 */
uint8_t currentFunction = 0;
CommandKey currentCommandKey; //! In addition to Saving current functionID active command key should be stored too

BleControlAction handlers[maxHandlers];

const bt_le_scan_param scan_params = {
    .type = BT_LE_SCAN_TYPE_PASSIVE,
    .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
    .interval = BT_GAP_SCAN_FAST_INTERVAL,
    .window = BT_GAP_SCAN_FAST_WINDOW,
    .timeout = 0,
    .interval_coded = 0,
    .window_coded = 0
};

Bluetooth::iBeacon i_beacon = { 0 };

/********************************************/
/* BLE connection */

atomic_t ads131m08NotificationsEnable = false;
atomic_t ads131m08_1_NotificationsEnable = false;
atomic_t max30102NotificationsEnable = false;
atomic_t mpu6050NotificationsEnable = false;
atomic_t bme280NotificationsEnable = false;
atomic_t rssiNotificationsEnable = false;
atomic_t qmc5883lNotificationsEnable = false;
atomic_t iBeaconNotificationsEnable = false;

/* BT832A Custom Service  */
bt_uuid_128 sensorServiceUUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));

/* Sensor Control Characteristic */
bt_uuid_128 sensorCtrlCharUUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0001cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));

/* Sensor Data Characteristic */
// ADS131M08 Data Pipe
bt_uuid_128 ads131DataUUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0002cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));
// MAX30102 Data Pipe
bt_uuid_128 max30102DataUUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0003cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));
// MPU6050 Data Pipe
bt_uuid_128 mpu6050DataUUID = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x0004cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));
// ADS131M08_1 Data Pipe
bt_uuid_128 ads131_1_DataUUID = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x0005cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));
// BME280 Data Pipe
bt_uuid_128 bme280DataUUID = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x0006cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));
// RSSI Data Pipe
bt_uuid_128 rssiDataUUID = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x0007cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));
// QMC5883L Data Pipe
bt_uuid_128 qmc5883lDataUUID = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x0008cafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));  
// BLE characteristic reserved for sending Control BLE commands
bt_uuid_128 controlUUID = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x0009cafe,  0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));
// iBeacons Data Pipe
bt_uuid_128 iBeaconUUID = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x000acafe,  0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000));

static ssize_t ControlCharacteristicWrite(bt_conn *conn, const bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

/**
 * @brief CCCD handler for ADS131M08 characteristic. Used to get notifications if client enables notifications
 *        for ADS131M08 characteristic. CCC = Client Characteristic Configuration
 * 
 * @param attr Ble Gatt attribute
 * @param value characteristic value
 */
static void ads131CccHandler(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	//notify_enable = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&ads131m08NotificationsEnable, value == BT_GATT_CCC_NOTIFY);
	LOG_DBG("ADS131M08 Notification %s", ads131m08NotificationsEnable ? "enabled" : "disabled");
}

/**
 * @brief CCCD handler for ADS131M08_1 characteristic. Used to get notifications if client enables notifications
 *        for ADS131M08 characteristic. CCC = Client Characteristic Configuration
 * 
 * @param attr Ble Gatt attribute
 * @param value characteristic value
 */
static void ads131_1_CccHandler(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	//notify_enable = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&ads131m08_1_NotificationsEnable, value == BT_GATT_CCC_NOTIFY);
	LOG_DBG("ADS131M08_1 Notification %s", ads131m08_1_NotificationsEnable ? "enabled" : "disabled");
}

/**
 * @brief CCCD handler for MAX30102 characteristic. Used to get notifications if client enables notifications
 *        for MAX30102 characteristic. CCC = Client Characteristic Configuration
 *
 * @param attr Ble Gatt attribute
 * @param value characteristic value
 */
static void max30102CccHandler(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	//notify_enable = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&max30102NotificationsEnable, value == BT_GATT_CCC_NOTIFY);
	LOG_DBG("Max30102 Notification %s", max30102NotificationsEnable ? "enabled" : "disabled");
}

/**
 * @brief CCCD handler for MPU6050 characteristic. Used to get notifications if client enables notifications
 *        for MPU6050 characteristic. CCC = Client Characteristic Configuration
 *
 * @param attr Ble Gatt attribute
 * @param value characteristic value
 */
static void mpu6050CccHandler(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	//notify_enable = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&mpu6050NotificationsEnable, value == BT_GATT_CCC_NOTIFY);
	LOG_DBG("MPU6050 Notification %s", mpu6050NotificationsEnable ? "enabled" : "disabled");
}

/**
 * @brief CCCD handler for QMC5883L characteristic. Used to get notifications if client enables notifications
 *        for QMC5883L characteristic. CCC = Client Characteristic Configuration
 *
 * @param attr Ble Gatt attribute
 * @param value characteristic value
 */
static void qmc5883lCccHandler(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	//notify_enable = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&qmc5883lNotificationsEnable, value == BT_GATT_CCC_NOTIFY);
	LOG_DBG("QMC5883L Notification %s", qmc5883lNotificationsEnable ? "enabled" : "disabled");
}

/**
 * @brief CCCD handler for iBeacons characteristic. Used to get notifications if client enables notifications
 *        for iBeacon characteristic. CCC = Client Characteristic Configuration
 *
 * @param attr Ble Gatt attribute
 * @param value characteristic value
 */
static void iBeaconCccHandler(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);	
    atomic_set(&iBeaconNotificationsEnable, value == BT_GATT_CCC_NOTIFY);
	LOG_DBG("iBeacon Notification %s", iBeaconNotificationsEnable ? "enabled" : "disabled");
}

/**
 * @brief CCCD handler for BME280 characteristic. Used to get notifications if client enables notifications
 *        for BME280 characteristic. CCC = Client Characteristic Configuration
 *
 * @param attr Ble Gatt attribute
 * @param value characteristic value
 */
static void bme280CccHandler(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	//notify_enable = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&bme280NotificationsEnable, value == BT_GATT_CCC_NOTIFY);
	LOG_DBG("BME280 Notification %s", bme280NotificationsEnable ? "enabled" : "disabled");
}

static void rssiCccHandler(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	//notify_enable = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&rssiNotificationsEnable, value == BT_GATT_CCC_NOTIFY);
	LOG_DBG("RSSI Notification %s", rssiNotificationsEnable ? "enabled" : "disabled");
    if(rssiNotificationsEnable){
        Bluetooth::RssiStartSampling();
    } else {
        Bluetooth::RssiStopSampling();
    }
}

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_LEN 12

/* Advertising data */
static uint8_t manuf_data[ADV_LEN] = {
	0x01 /*SKD version */,
	0x83 /* STM32WB - P2P Server 1 */,
	0x00 /* GROUP A Feature  */,
	0x00 /* GROUP A Feature */,
	0x00 /* GROUP B Feature */,
	0x00 /* GROUP B Feature */,
	0x00, /* BLE MAC start -MSB */
	0x00,
	0x00,
	0x00,
	0x00,
	0x00, /* BLE MAC stop */
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)
};

/**
 * @brief Gatt Service definition
 */
BT_GATT_SERVICE_DEFINE(bt832a_svc,
BT_GATT_PRIMARY_SERVICE(&sensorServiceUUID),                                            // 0
BT_GATT_CHARACTERISTIC(&sensorCtrlCharUUID.uuid,                                        // 1
		        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,                    // 2, 3
		        BT_GATT_PERM_WRITE, nullptr, ControlCharacteristicWrite, nullptr),
BT_GATT_CHARACTERISTIC(&ads131DataUUID.uuid, BT_GATT_CHRC_NOTIFY,                       // 4, 5
		        BT_GATT_PERM_READ, nullptr, nullptr, nullptr), //&ble_tx_buff),
BT_GATT_CCC(ads131CccHandler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),                  // 6
BT_GATT_CHARACTERISTIC(&max30102DataUUID.uuid, BT_GATT_CHRC_NOTIFY,                     // 7,8
		        BT_GATT_PERM_READ, nullptr, nullptr, nullptr),
BT_GATT_CCC(max30102CccHandler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),                // 9
BT_GATT_CHARACTERISTIC(&mpu6050DataUUID.uuid, BT_GATT_CHRC_NOTIFY,                      // 10, 11
		        BT_GATT_PERM_READ, nullptr, nullptr, nullptr),
BT_GATT_CCC(mpu6050CccHandler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),                 // 12
BT_GATT_CHARACTERISTIC(&ads131_1_DataUUID.uuid, BT_GATT_CHRC_NOTIFY,                    // 13, 14
		        BT_GATT_PERM_READ, nullptr, nullptr, nullptr),
BT_GATT_CCC(ads131_1_CccHandler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),               // 15
BT_GATT_CHARACTERISTIC(&bme280DataUUID.uuid, BT_GATT_CHRC_NOTIFY,                       // 16, 17
		        BT_GATT_PERM_READ, nullptr, nullptr, nullptr),
BT_GATT_CCC(bme280CccHandler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),                  // 18
BT_GATT_CHARACTERISTIC(&rssiDataUUID.uuid, BT_GATT_CHRC_NOTIFY,                         // 19, 20
		        BT_GATT_PERM_READ, nullptr, nullptr, nullptr),
BT_GATT_CCC(rssiCccHandler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),                    // 21
BT_GATT_CHARACTERISTIC(&qmc5883lDataUUID.uuid, BT_GATT_CHRC_NOTIFY,                     // 22, 23
		        BT_GATT_PERM_READ, nullptr, nullptr, nullptr),
BT_GATT_CCC(qmc5883lCccHandler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),                // 24
BT_GATT_CHARACTERISTIC(&iBeaconUUID.uuid, BT_GATT_CHRC_NOTIFY,                          // 25, 26
		        BT_GATT_PERM_READ, nullptr, nullptr, nullptr),
BT_GATT_CCC(iBeaconCccHandler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),                 // 27
BT_GATT_CHARACTERISTIC(&controlUUID.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
    BT_GATT_PERM_WRITE, nullptr, ControlCharacteristicWrite, nullptr),
);

/********************************************************/


/**
 * @brief Callback clled when Bluetooth is initialized. Starts BLE server
 *
 * @param err bluetooth initialization error code
 */
void OnBluetoothStarted(int err)
{
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);

    LOG_DBG("Bluetooth initialized");

    /* Start advertising */
    err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), nullptr, 0); // nullptr for scan response 0 for scan response data size
    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_DBG("Configuration mode: waiting connections...");
}

/**
 * @brief Callback function called when client(master) sends Gatt characteristic write command
 *
 * @param conn connection
 * @param attr GATT attribute
 * @param buf  inbound buffer
 * @param len  inbound buffer size
 * @param offset current transfer offset. used when write was splitted into several BLE packets
 * @param flags flags
 * @return ssize_t number of bytes processed. usually equal to number of received bytes.
 */
ssize_t ControlCharacteristicWrite(bt_conn *conn, const bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint16_t retval = len;
    const uint8_t* buffer = static_cast<const uint8_t*>(buf);

    LOG_DBG("%d Bytes received!", len);
    LOG_DBG("Offset: %d", offset);
    LOG_DBG("Flags: 0x%X", flags);
    LOG_DBG("Data[0]: 0x%X", *buffer);

    // new message (could be partial)
    if (offset == 0)
    {
        if (len < controlHeaderSize)
        {
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }

        // extract command Id and skip BLE frame header
        currentFunction = *buffer;

        // copy command key into struct directly to avoid possible alignment issues
        currentCommandKey.key[0] = *(buffer + 1);
        currentCommandKey.key[1] = *(buffer + 2);

        buffer += controlHeaderSize;
        len -= controlHeaderSize;
    }
    else
    {
        offset -= controlHeaderSize;
    }

    if (handlers[currentFunction] != nullptr)
    {
        handlers[currentFunction](buffer, currentCommandKey, BleLength{len}, BleOffset {offset});
    }

    return retval;
}

static void onBleDeviceFound(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
    uint8_t *p_adv_data;
    uint16_t i;
    
    if (type == BT_GAP_ADV_TYPE_ADV_NONCONN_IND){
        uint8_t ad_length, ad_type;
        p_adv_data  = &ad->data[0];

        i = 0;
        while(i < ad->len)
        {
            ad_length = p_adv_data[i];
            ad_type = p_adv_data[i + 1];

            switch (ad_type)
            {
                case BT_DATA_FLAGS:
                    break;
                case BT_DATA_TX_POWER:
                    break;
                case BT_DATA_NAME_COMPLETE:
                {

                }
                case BT_DATA_MANUFACTURER_DATA:
                {
                    if ((ad_length == IBEACON_MAX_LENGTH + 5) &&
                      (p_adv_data[i + 4] == IBEACON_TYPE) && // 0x02 => iBeacon Type
                      (p_adv_data[i + 5] == IBEACON_MAX_LENGTH))  // iBeacon Length is 0x15 (21 Bytes)
                    {
                        /* iBeacon device found. */
                        memcpy(&i_beacon.addr[0], addr->a.val, BT_ADDR_SIZE);
                        i_beacon.rssi = rssi;
                        memcpy(&i_beacon.uuid[0], &p_adv_data[i + 6], 16);
                        memcpy(&i_beacon.major[0], &p_adv_data[i + 22], 2);
                        memcpy(&i_beacon.minor[0], &p_adv_data[i + 24], 2);
                        i_beacon.tx_pwr = p_adv_data[i + 26];
                        /* Send Notification over iBeacon Characteristic */
                        if (atomic_get(&iBeaconNotificationsEnable)){
                            bt_gatt_notify(nullptr, &bt832a_svc.attrs[CharacteristiciBeaconData], &i_beacon, sizeof(i_beacon));
                        }
                        
                        LOG_HEXDUMP_DBG(addr->a.val, BT_ADDR_SIZE, "BLE_addr");
                        LOG_DBG("RSSI: %d", rssi);
                        LOG_HEXDUMP_DBG(&p_adv_data[i + 6], 16, "iBeacon_UUID");
                        LOG_HEXDUMP_DBG(&p_adv_data[i + 22], 2, "iBeacon_Major");
                        LOG_HEXDUMP_DBG(&p_adv_data[i + 24], 2, "iBeacon_Minor");
                        LOG_HEXDUMP_DBG(&p_adv_data[i + 26], 1, "iBeacon_Tx_Power");
                    }
                    break;
                }

                default:
                    break;
            }/* end of switch */
            i += ad_length + 1; /* increment the iterator to go on next element */

        }/* end of while */
    } /* end of if(type ==) */

}

void StartBeaconScanning(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(&scan_params, onBleDeviceFound);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

void StopBeaconScanning(void)
{
	int err;
 
	/* This demo doesn't require active scan */
	err = bt_le_scan_stop();
	if (err) {
		printk("Scanning failed to stop (err %d)\n", err);
		return;
	}

	printk("Scanning successfully stopped\n");
}

/**
 * @brief Register Control callback
 * 
 * @param commandId command ID
 * @param action Action to call when command ith commandId is received via BLE
 */
void GattSetControlCallback(CommandId commandId, BleControlAction&& action)
{
    if (handlers[static_cast<size_t>(commandId)] == nullptr)
    {
        handlers[static_cast<size_t>(commandId)] = std::move(action);
    }
    else
    {
        LOG_ERR("Handler with id %d is already registred", static_cast<int>(commandId));
    }
}

} // namespace Bluetooth::Gatt
