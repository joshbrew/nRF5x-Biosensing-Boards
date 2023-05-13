#include <zephyr.h>

#include <bluetooth/gatt.h>
#include <bluetooth/uuid.h>
#include "ble_service.hpp"
#include <functional>

#include <logging/log.h>

#include <sys/atomic.h>

#include "ble_gatt.hpp"
#include "qmc5883l.hpp"

namespace Bluetooth::Gatt
{

LOG_MODULE_REGISTER(BleGatt);

/********************************************/
/* BLE connection */

atomic_t ads131m08NotificationsEnable = false;
atomic_t ads131m08_1_NotificationsEnable = false;
atomic_t max30102NotificationsEnable = false;
atomic_t mpu6050NotificationsEnable = false;
atomic_t bme280NotificationsEnable = false;
atomic_t rssiNotificationsEnable = false;
atomic_t qmc5883lNotificationsEnable = false;

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
	LOG_INF("ADS131M08 Notification %s", ads131m08NotificationsEnable ? "enabled" : "disabled");
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
	LOG_INF("ADS131M08_1 Notification %s", ads131m08_1_NotificationsEnable ? "enabled" : "disabled");
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
	LOG_INF("Max30102 Notification %s", max30102NotificationsEnable ? "enabled" : "disabled");
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
	LOG_INF("MPU6050 Notification %s", mpu6050NotificationsEnable ? "enabled" : "disabled");
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
	LOG_INF("QMC5883L Notification %s", qmc5883lNotificationsEnable ? "enabled" : "disabled");
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
	LOG_INF("BME280 Notification %s", bme280NotificationsEnable ? "enabled" : "disabled");
}

static void rssiCccHandler(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	//notify_enable = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&rssiNotificationsEnable, value == BT_GATT_CCC_NOTIFY);
	LOG_INF("RSSI Notification %s", rssiNotificationsEnable ? "enabled" : "disabled");
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

    LOG_INF("Bluetooth initialized");

    /* Start advertising */
    err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), nullptr, 0); // nullptr for scan response 0 for scan response data size
    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Configuration mode: waiting connections...");
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
static ssize_t ControlCharacteristicWrite(bt_conn *conn, const bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint16_t retval = len;
    const uint8_t* buffer = static_cast<const uint8_t*>(buf);

    LOG_INF("%d Bytes received!", len);
    LOG_INF("Offset: %d", offset);
    LOG_INF("Flags: 0x%X", flags);
    LOG_INF("Data[0]: 0x%X", *buffer);

    return retval;
}

} // namespace Bluetooth::Gatt
