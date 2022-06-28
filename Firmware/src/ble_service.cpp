#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <logging/log.h>
#include <sys/atomic.h>

#include "ble_gatt.hpp"
#include "ble_service.hpp"


LOG_MODULE_REGISTER(bluetooth);

namespace
{

constexpr static uint16_t connectionIntervalMin = 6; ///< Minimal connection interval in 1.25 milliseconds intervals
constexpr static uint16_t connectionIntervalMax = 8; ///< Maximum connection interval in 1.25 milliseconds intervals
constexpr static uint16_t connectionLatency = 0;     ///< Connection latancy
constexpr static uint16_t connectionTimeout = 400;   ///< Connection timeout in 10 msec intervals

//Bluetooth::Gatt::BleOutputWorker worker;   ///< Ble output characteristic worker
bt_conn *activeConnection = nullptr;       ///< Active connection

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
    atomic_set(&Bluetooth::Gatt::max30102NotificationsEnable, false);    
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

    if(interval != 6){ /* If connection interval is greater than 6 x 1.25ms = 7.5ms */
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
    }

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
    return err;
}

void Ads131m08Notify(const uint8_t* data, const uint8_t len)
{
    if (atomic_get(&Gatt::ads131m08NotificationsEnable))
    {
        bt_gatt_notify(nullptr, &Gatt::bt832a_svc.attrs[Gatt::CharacteristicAds131Data], data, len);
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

} // namespace Bluetooth
