#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_vs.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <logging/log.h>
#include <sys/atomic.h>
#include "ble_commands.hpp"

#include "ble_gatt.hpp"
#include "ble_service.hpp"
#include <mgmt/mcumgr/smp_bt.h>


LOG_MODULE_REGISTER(bluetooth);

namespace
{

constexpr static auto RssiPollPeriod = 333;              ///< RSSI polling period in milliseconds
constexpr static uint16_t connectionIntervalMin = 6; ///< Minimal connection interval in 1.25 milliseconds intervals
constexpr static uint16_t connectionIntervalMax = 12; ///< Maximum connection interval in 1.25 milliseconds intervals //500/9 = 56.25 packets + 5 + 3 = 60 packets/sec.  1000/60 = 16.67 ms/packet. 16.67 / 1.25 = 12.5 connectionIntervalMax
constexpr static uint16_t connectionLatency = 0;     ///< Connection latancy
constexpr static uint16_t connectionTimeout = 400;   ///< Connection timeout in 10 msec intervals

//Bluetooth::Gatt::BleOutputWorker worker;   ///< Ble output characteristic worker
bt_conn *activeConnection = nullptr;       ///< Active connection
static uint16_t default_conn_handle = 0;

/**
 * @brief Callback called when MTU paramter is updated with bt_gatt_exchange_mtu() function
 */
void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
 {
    struct bt_conn_info info = {0};

    printk("MTU exchange %s\n", err == 0 ? "successful" : "failed");
    err = bt_conn_get_info(conn, &info);

    if (info.role == BT_CONN_ROLE_MASTER) {

    }
}

/**
 * @brief Callback called when new client is connected
 *
 * @param connected connected bluetooth connection
 * @param err connection error
 */
void OnClientConnected(bt_conn *connected, uint8_t err)
{
    int ret;

    if (err)
    {
        LOG_ERR("Connection failed (err %u)", err);
    }
    else
    {
        LOG_INF("Connected");

        if ((!activeConnection) && err == 0)
        {
            activeConnection = bt_conn_ref(connected);
            ret = bt_hci_get_conn_handle(activeConnection, &default_conn_handle);
            if(ret){
                LOG_ERR("No connection handle. Err: %d", ret);
            }
            /* Set MTU parameter. NOTE: It's allowed to do it only once during a connection */
            static struct bt_gatt_exchange_params exchange_params;
            exchange_params.func = exchange_func;

            int error = bt_gatt_exchange_mtu(activeConnection, &exchange_params);

            bt_conn_le_phy_param phy_param = BT_CONN_LE_PHY_PARAM_INIT(
            BT_GAP_LE_PHY_2M,
            BT_GAP_LE_PHY_2M
            );
            error = bt_conn_le_phy_update(connected, &phy_param);

            if (error){
                LOG_ERR("Failed to update BLE PHY parameters (err = %d)\n", error);
            } else {
                LOG_INF("BLE PHY updated!");
            }
        }
    }
}

/**
 * @brief Callback called when client is disconnected. Used to stop BLE notification if no clients are connected.
 *
 * @param disconn disconnected bluetooth connection
 * @param reason disconnection reason
 */
void OnClientDisconnected(struct bt_conn *disconn, uint8_t reason)
{
    if (activeConnection)
    {
        bt_conn_unref(activeConnection);
        activeConnection = nullptr;

    }
    atomic_set(&Bluetooth::Gatt::ads131m08NotificationsEnable, false);
    atomic_set(&Bluetooth::Gatt::ads131m08_1_NotificationsEnable, false);
    atomic_set(&Bluetooth::Gatt::max30102NotificationsEnable, false);
    atomic_set(&Bluetooth::Gatt::mpu6050NotificationsEnable, false);
    atomic_set(&Bluetooth::Gatt::bme280NotificationsEnable, false);
    atomic_set(&Bluetooth::Gatt::rssiNotificationsEnable, false);
    atomic_set(&Bluetooth::Gatt::qmc5883lNotificationsEnable, false);
    atomic_set(&Bluetooth::Gatt::iBeaconNotificationsEnable, false);    
    LOG_INF("Disconnected (reason %u)", reason);
}

/**
 * @brief This callback notifies the application that a remote device
 *        is requesting to update the connection parameters.
 *
 *  @param conn Connection object.
 *  @param param Proposed connection parameters.
 *  @return true to accept the parameters, or false to reject them.
 */
bool OnLeParamUpdateRequest(struct bt_conn *conn,
			     struct bt_le_conn_param *param)
{

    LOG_INF("LE Connection parameters requested!");
    LOG_INF("Min Connection Interval: %d x 1.25ms", param->interval_min);
    LOG_INF("Max Connection Interval: %d x 1.25ms", param->interval_max);
    return true;
}

/** @brief The parameters for an LE connection have been updated.
 *
 *  This callback notifies the application that the connection
 *  parameters for an LE connection have been updated.
 *
 *  @param conn Connection object.
 *  @param interval Connection interval.
 *  @param latency Connection latency.
 *  @param timeout Connection supervision timeout.
 */
void OnLeParamUpdated(struct bt_conn *conn, uint16_t interval,
				 uint16_t latency, uint16_t timeout)
{
    LOG_INF("LE connection parameters updated!");
    LOG_INF("Connection interval: %d x 1.25ms", interval);
    int error;

    //if(interval > connectionIntervalMax || interval < connectionIntervalMin){ /* If connection interval is greater than 6 x 1.25ms = 7.5ms */
        bt_le_conn_param param = BT_LE_CONN_PARAM_INIT(
            connectionIntervalMin,
            connectionIntervalMax,
            connectionLatency,
            connectionTimeout);
        error = bt_conn_le_param_update(conn, &param);

        if (error){
            LOG_ERR("Failed to update connection parameters (err = %d)\n", error);
            //return;
        } else {
            LOG_INF("Connection parameters successfully updated!");
        }
    //}

}

/** @brief The PHY of the connection has changed.
 *
 *  This callback notifies the application that the PHY of the
 *  connection has changed.
 *
 *  @param conn Connection object.
 *  @param info Connection LE PHY information.
 */
void OnPhyUpdated(struct bt_conn *conn,
			     struct bt_conn_le_phy_info *param)
{

    LOG_INF("LE PHY Updated!");
    LOG_INF("TX PHY: %d", param->tx_phy);
    LOG_INF("RX PHY: %d", param->rx_phy);
}

/**
 * @brief connection status callback
 */
bt_conn_cb connectionCallbacks =
{
    .connected = OnClientConnected,
    .disconnected = OnClientDisconnected,
    .le_param_req = OnLeParamUpdateRequest,
    .le_param_updated = OnLeParamUpdated,
    .le_phy_updated = OnPhyUpdated,
};

} // namespace

namespace Bluetooth
{

    constexpr static int stackSize = 1024;           ///< Worker thread size
    constexpr static int taskPriority = 7;           ///< Worker thread priority
    K_THREAD_STACK_DEFINE(pollStackArea, stackSize); ///< Worker thread stack
    k_sem rssiPollSemaphore;    ///< Semaphore used for RSSI polling
    k_thread worker;            ///< Worker thread
    k_timer rssiPollTimer;      ///< Timer object

    static void RssiNotify(const int8_t* data, const uint8_t len);
    static void WorkingThread(void *, void *, void *);
    static void RssiPollTimerHandler(k_timer *tmr);

    /**
     * @brief Main working thread. Used to perform RSSI polling.
     *
     */
    static void WorkingThread(void *, void *, void *){
        int8_t rssi[1] = {};

        for (;;)
        {
            k_sem_take(&rssiPollSemaphore, K_FOREVER);
            //LOG_INF("Time to get RSSI!");
            Bluetooth::read_conn_rssi(rssi);
            Bluetooth::RssiNotify(rssi, 1);
        }
    }

    /**
     * @brief Timer handler. Used to queue Reading of the signal strength (RSSI) data.
     *
     * @param tmr timer object
     * @warning Called at ISR Level, no actual workload should be implemented here
     */
    static void RssiPollTimerHandler(k_timer *tmr){
        k_sem_give(&rssiPollSemaphore);
    }

/**
 * @brief Function used to setup BLE Service
 *
 * @return BLE error code
 */
int SetupBLE()
{
    //worker.Initialize();

    bt_conn_cb_register(&connectionCallbacks);

    int err = bt_enable(&Gatt::OnBluetoothStarted);
    if (err)
    {
        LOG_INF("enable Bluetooth with status %d", err);
    }

    GattRegisterControlCallback(CommandId::BleCmd, OnBleCommand);

	/* Initialize the Bluetooth mcumgr transport. */
	smp_bt_register();

    k_timer_init(&rssiPollTimer, RssiPollTimerHandler, nullptr);
    k_sem_init(&rssiPollSemaphore, 0, 1);
    k_thread_create(&worker, pollStackArea, K_THREAD_STACK_SIZEOF(pollStackArea),
                &WorkingThread, nullptr, nullptr, nullptr, taskPriority, 0, K_NO_WAIT);

    return err;
}

bool OnBleCommand(const uint8_t *buffer, CommandKey key, BleLength length, BleOffset offset){
    if (offset.value != 0 || length.value == 0)
    {
        return false;
    }
    LOG_DBG("BLE Command received");

    CommandKey bleCommand;
    memcpy(&bleCommand, &key, sizeof(key));
          
    switch(bleCommand.key[0]){
        case static_cast<uint8_t>(BleCommand::StartBeaconScan):
            Gatt::StartBeaconScanning();
            break;
        case static_cast<uint8_t>(BleCommand::StopBeaconScan):
            Gatt::StopBeaconScanning();
            break;
        
        default:
            break;
    }
    
    return true;
}

void RssiStartSampling(){
    // Start BME280 polling
    k_timer_start(&rssiPollTimer, K_MSEC(RssiPollPeriod), K_MSEC(RssiPollPeriod));
}

void RssiStopSampling(){
    k_timer_stop(&rssiPollTimer);
}

void Ads131m08Notify(const uint8_t* data, const uint8_t len)
{
    if (atomic_get(&Gatt::ads131m08NotificationsEnable))
    {
        bt_gatt_notify(nullptr, &Gatt::bt832a_svc.attrs[Gatt::CharacteristicAds131Data], data, len);
    }
}

void Ads131m08_1_Notify(const uint8_t* data, const uint8_t len)
{
    if (atomic_get(&Gatt::ads131m08_1_NotificationsEnable))
    {
        bt_gatt_notify(nullptr, &Gatt::bt832a_svc.attrs[Gatt::CharacteristicAds131_1_Data], data, len);
    }
}

/**
 * @brief Send BLE notification through MAX30102 Data Pipe.
 *
 * @param data pointer to datasource containing MAX30102 data samples
 * @param len  the number of samples to transfer
 */
void Max30102Notify(const uint8_t* data, const uint8_t len)
{
    if (atomic_get(&Gatt::max30102NotificationsEnable))
    {
        bt_gatt_notify(nullptr, &Gatt::bt832a_svc.attrs[Gatt::CharacteristicMax30102Data], data, len);
    }
}

/**
 * @brief Send BLE notification through MPU6050 Data Pipe.
 *
 * @param data pointer to datasource containing MAX30102 data samples
 * @param len  the number of samples to transfer
 */
void Mpu6050Notify(const uint8_t* data, const uint8_t len)
{
    if (atomic_get(&Gatt::mpu6050NotificationsEnable))
    {
        bt_gatt_notify(nullptr, &Gatt::bt832a_svc.attrs[Gatt::CharacteristicMpu6050Data], data, len);
    }
}

/**
 * @brief Send BLE notification through QMC5883L Data Pipe.
 *
 * @param data pointer to datasource containing MAX30102 data samples
 * @param len  the number of samples to transfer
 */
void Qmc5883lNotify(const uint8_t* data, const uint8_t len)
{
    if (atomic_get(&Gatt::qmc5883lNotificationsEnable))
    {
        bt_gatt_notify(nullptr, &Gatt::bt832a_svc.attrs[Gatt::CharacteristicQmc5883lData], data, len);
    }
}

/**
 * @brief Send BLE notification through BME280 Data Pipe.
 *
 * @param data pointer to datasource containing BME280 data samples
 * @param len  the number of samples to transfer
 */
void Bme280Notify(const uint8_t* data, const uint8_t len)
{
    //LOG_HEXDUMP_INF(data, len, "bme280");
    if (atomic_get(&Gatt::bme280NotificationsEnable))
    {
        bt_gatt_notify(nullptr, &Gatt::bt832a_svc.attrs[Gatt::CharacteristicBme280Data], data, len);
    }
}

/**
 * @brief Send BLE notification through RSSI Data Pipe.
 *
 * @param data pointer to datasource containing RSSI data
 * @param len  the number of samples to transfer
 */
static void RssiNotify(const int8_t* data, const uint8_t len)
{
    if (atomic_get(&Gatt::rssiNotificationsEnable))
    {
        bt_gatt_notify(nullptr, &Gatt::bt832a_svc.attrs[Gatt::CharacteristicRssiData], data, len);
    }
}

void read_conn_rssi(int8_t *rssi)
{
	struct net_buf *buf, *rsp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;

	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
	if (!buf) {
		LOG_ERR("Unable to allocate command buffer\n");
		return;
	}

	cp = (bt_hci_cp_read_rssi *) (net_buf_add(buf, sizeof(*cp)));
	cp->handle = default_conn_handle; //activeConnection->handle; //sys_cpu_to_le16(activeConnection->handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_read_rssi *)rsp->data)->status : 0;
		LOG_ERR("Read RSSI err: %d reason 0x%02x\n", err, reason);
		return;
	}

	rp = (bt_hci_rp_read_rssi *)rsp->data;
	*rssi = rp->rssi;
    //LOG_INF("Connected (%d) - RSSI = %d", default_conn_handle, *rssi);

	net_buf_unref(rsp);
}

/**
 * @brief Register Control callback
 * 
 * @param commandId command ID
 * @param action Action to call when command ith commandId is received via BLE
 */
void GattRegisterControlCallback(CommandId commandId, BleControlAction&& action)
{
    Gatt::GattSetControlCallback(commandId, std::forward<BleControlAction>(action));
}

} // namespace Bluetooth
