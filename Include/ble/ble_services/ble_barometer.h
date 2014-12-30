


#ifndef BLE_BAROMETER_H__
#define BLE_BAROMETER_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"


// Own UUID base and adlias.
#define BAROMETER_UUID_BASE {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00};
#define BLE_UUID_BAROMETER_SERVICE			0x1110
#define BLE_UUID_BAROMETER_LEVEL_CHAR		0x1111


/**@brief Barometer Service event type. */
typedef enum
{
    BLE_BAROMETER_EVT_NOTIFICATION_ENABLED,                             /**< Battery value notification enabled event. */
    BLE_BAROMETER_EVT_NOTIFICATION_DISABLED                             /**< Battery value notification disabled event. */
} ble_barometer_evt_type_t;

/**@brief Barometer Service event. */
typedef struct
{
    ble_barometer_evt_type_t evt_type;                                  /**< Type of event. */
} ble_barometer_evt_t;

typedef struct ble_barometer_meas_s
{
    int16_t barometer_measurement;
} ble_barometer_meas_t;

// Forward declaration of the ble_barometer_t type. 
typedef struct ble_barometer_s ble_barometer_t;

/**@brief Barometer Service event handler type. */
typedef void (*ble_barometer_evt_handler_t) (ble_barometer_t * p_barometer, ble_barometer_evt_t * p_evt);

/**@brief Barometer Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_barometer_evt_handler_t   evt_handler;                    /**< Event handler to be called for handling events in the Barometer Service. */
    bool                          support_notification;           /**< TRUE if notification of Barometer Level measurement is supported. */
    ble_srv_report_ref_t *        p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Barometer characteristic */
    uint8_t                       initial_barometer_level;        /**< Initial barometer level */
    ble_srv_cccd_security_mode_t  barometer_level_char_attr_md;     /**< Initial security level for barometer characteristics attribute */
    ble_gap_conn_sec_mode_t       barometer_level_report_read_perm; /**< Initial security level for barometer report read attribute */
} ble_barometer_init_t;

/**@brief Barometer Service structure. This contains various status information for the service. */
typedef struct ble_barometer_s
{
    ble_barometer_evt_handler_t   evt_handler;                    /**< Event handler to be called for handling events in the Barometer Service. */
    uint16_t                      service_handle;                 /**< Handle of Barometer Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      barometer_level_handles;        /**< Handles related to the Barometer Level characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       barometer_level_last;           /**< Last Barometer Level measurement passed to the Battery Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of Barometer Level is supported. */
		uint8_t 											uuid_type;
} ble_barometer_t;

/**@brief Function for initializing the Barometer Service.
 *
 * @param[out]  p_barometer       Barometer Service structure. This structure will have to be supplied by
 *                                the application. It will be initialized by this function, and will later
 *                                be used to identify this particular service instance.
 * @param[in]   p_barometer_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_barometer_init(ble_barometer_t * p_barometer, const ble_barometer_init_t * p_barometer_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Barometer Service.
 *
 * @param[in]   p_barometer     Barometer Service structure.
 * @param[in]   p_ble_evt       Event received from the BLE stack.
 */
void ble_barometer_on_ble_evt(ble_barometer_t * p_barometer, ble_evt_t * p_ble_evt);

/**@brief Function for updating the barometer level.
 *
 * @details The application calls this function after having performed a barometer measurement. If
 *          notification has been enabled, the barometer level characteristic is sent to the client.
 *
 * @param[in]   p_barometer         Barometer Service structure.
 * @param[in]   barometer_level     New barometer measurement value
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
//uint32_t ble_barometer_level_update(ble_barometer_t * p_barometer, uint16_t barometer_level);
uint32_t ble_barometer_send(ble_barometer_t * p_barometer, ble_barometer_meas_t * p_barometer_measurement);

#endif // BLE_BAROMETER_H__

/** @} */
