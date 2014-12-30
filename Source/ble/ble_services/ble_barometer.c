/*
*
*/

#include "ble_barometer.h"
#include <string.h>
#include "ble_l2cap.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


#define INVALID_BAROMETER_LEVEL 255
#define OPCODE_LENGTH  1                                                    /**< Length of opcode inside Cycling Power Measurement packet. */
#define HANDLE_LENGTH  2 
#define MAX_BAROMETER_LEN				(BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_barometer Barometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_barometer_t * p_barometer, ble_evt_t * p_ble_evt)
{
    p_barometer->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_barometer       Barometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_barometer_t * p_barometer, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_barometer->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_barometer       Barometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_meas_cccd_write(ble_barometer_t * p_barometer, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_barometer->evt_handler != NULL)
        {
            ble_barometer_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_BAROMETER_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_BAROMETER_EVT_NOTIFICATION_DISABLED;
            }

            p_barometer->evt_handler(p_barometer, &evt);
        }
    }
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_barometer       Barometer Service structure.
 * @param[in]   p_ble_evt   			Event received from the BLE stack.
 */
static void on_write(ble_barometer_t * p_barometer, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_barometer->barometer_level_handles .cccd_handle)
    {
        on_meas_cccd_write(p_barometer, p_evt_write);
    }
}



void ble_barometer_on_ble_evt(ble_barometer_t * p_barometer, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_barometer, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_barometer, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_barometer, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a Barometer Level Measurement.
 *
 * @param[in]   p_barometer        				Barometer Service structure.
 * @param[in]   p_barometer_measurement   Measurement to be encoded.
 * @param[out]  p_encoded_buffer    			Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t barometer_measurement_encode(ble_barometer_t *       p_barometer,
                                     ble_barometer_meas_t *  p_barometer_measurement,
                                     uint8_t *          p_encoded_buffer)
{
    uint16_t flags = 0;
    uint8_t len   = 2;

    flags = 0x01;
    //len += uint16_encode(p_barometer_measurement->barometer_measurement, &p_encoded_buffer[len]);
		//len = 20;    ---- CHANGED ----
    // Flags Field
    //p_encoded_buffer[0] = (uint8_t) ((flags >> 8) & 0x00FF);
    //p_encoded_buffer[1] = (uint8_t) (flags & 0x00FF);
		p_encoded_buffer[0] = (uint8_t) ((p_barometer_measurement->barometer_measurement)>>8);
		p_encoded_buffer[1] = (uint8_t) ((p_barometer_measurement->barometer_measurement));
		len = 2;
	
    return len;
}

/**@brief Function for adding the Barometer Level characteristic.
 *
 * @param[in]   p_barometer        Barometer Service structure.
 * @param[in]   p_barometer_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t barometer_level_char_add(ble_barometer_t * p_barometer, const ble_barometer_init_t * p_barometer_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
		ble_barometer_meas_t initial_barometer_level;
    uint8_t             encoded_barometer[MAX_BAROMETER_LEN];

		memset(&cccd_md, 0, sizeof(cccd_md));

		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		cccd_md.write_perm = p_barometer_init->barometer_level_char_attr_md.cccd_write_perm;
		cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    //char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BAROMETER_LEVEL_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm );
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = barometer_measurement_encode(p_barometer, &initial_barometer_level, encoded_barometer);;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_BAROMETER_LEN;
    attr_char_value.p_value   = encoded_barometer;

    return sd_ble_gatts_characteristic_add(p_barometer->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_barometer->barometer_level_handles);
}


uint32_t ble_barometer_init(ble_barometer_t * p_barometer, const ble_barometer_init_t * p_barometer_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
	
		p_barometer->uuid_type = BLE_UUID_TYPE_VENDOR_BEGIN;

		ble_uuid128_t base_uuid = BAROMETER_UUID_BASE;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &p_barometer->uuid_type);
		    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		ble_uuid.type = p_barometer->uuid_type;
		ble_uuid.uuid = BLE_UUID_BAROMETER_SERVICE;
		
	
    // Initialize service structure
    p_barometer->evt_handler               = p_barometer_init->evt_handler;
    p_barometer->conn_handle               = BLE_CONN_HANDLE_INVALID;
    //p_barometer->is_notification_supported = p_barometer_init->support_notification;
    //p_barometer->barometer_level_last      = INVALID_BAROMETER_LEVEL;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BAROMETER_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_barometer->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add barometer level characteristic
    return barometer_level_char_add(p_barometer, p_barometer_init);
}


uint32_t ble_barometer_send(ble_barometer_t * p_barometer, ble_barometer_meas_t * p_barometer_measurement)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_barometer->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_barometer_meas[2];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = barometer_measurement_encode(p_barometer, p_barometer_measurement, encoded_barometer_meas);
        hvx_len = len;		// Length = 20 Bytes!!!!

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle   = p_barometer->barometer_level_handles.value_handle;
        hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset   = 0;
        hvx_params.p_len    = &hvx_len;
        hvx_params.p_data   = encoded_barometer_meas;

        err_code = sd_ble_gatts_hvx(p_barometer->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


/*
uint32_t ble_barometer_level_update(ble_barometer_t * p_barometer, uint16_t barometer_level)
{
    uint32_t err_code = NRF_SUCCESS;

    if (barometer_level != p_barometer->barometer_level_last)
    {
        uint16_t len = sizeof(uint8_t);

        // Save new barometer value
        p_barometer->barometer_level_last = barometer_level;

        // Update databarometere
        err_code = sd_ble_gatts_value_set(p_barometer->barometer_level_handles.value_handle,
                                          0,
                                          &len,
                                          &barometer_level);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_barometer->conn_handle != BLE_CONN_HANDLE_INVALID) && p_barometer->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(uint8_t);

            hvx_params.handle = p_barometer->barometer_level_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &len;
            hvx_params.p_data = &barometer_level;

            err_code = sd_ble_gatts_hvx(p_barometer->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}*/
