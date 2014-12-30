/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/* Attention!
*  To maintain compliance with Nordic Semiconductor ASA�s Bluetooth profile
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_cp.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"

#define OPCODE_LENGTH  1                                                    /**< Length of opcode inside Cycling Power Measurement packet. */
#define HANDLE_LENGTH  2                                                    /**< Length of handle inside Cycling Power Measurement packet. */
#define MAX_CPM_LEN   (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted Cycling Power Measurement. */

// Cycling Power Measurement flag bits
#define CP_MEAS_FLAG_MASK_WHEEL_REV_DATA_PRESENT   (0x01 << 0)             /**< Wheel revolution data present flag bit. */
#define CP_MEAS_FLAG_MASK_CRANK_REV_DATA_PRESENT   (0x01 << 1)             /**< Crank revolution data present flag bit. */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cps      Cycling Power Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cps_t * p_cps, ble_evt_t * p_ble_evt)
{
    p_cps->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cps      Cycling Power Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cps_t * p_cps, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cps->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling write events to the cpS Measurement characteristic.
 *
 * @param[in]   p_cps        Cycling Power Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_meas_cccd_write(ble_cps_t * p_cps, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_cps->evt_handler != NULL)
        {
            ble_cps_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_cpS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_cpS_EVT_NOTIFICATION_DISABLED;
            }

            p_cps->evt_handler(p_cps, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cps      Cycling Power Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cps_t * p_cps, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_cps->meas_handles.cccd_handle)
    {
        on_meas_cccd_write(p_cps, p_evt_write);
    }
}

void ble_cps_on_ble_evt(ble_cps_t * p_cps, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_cps, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_cps, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_cps, p_ble_evt);
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for encoding a cpS Measurement.
 *
 * @param[in]   p_cps              Cycling Power Service structure.
 * @param[in]   p_cp_measurement   Measurement to be encoded.
 * @param[out]  p_encoded_buffer    Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t cp_measurement_encode(ble_cps_t *       p_cps,
                                     ble_cps_meas_t *  p_cp_measurement,
                                     uint8_t *          p_encoded_buffer)
{
    uint16_t flags = 0;
    uint8_t len   = 2;

    flags = 0x01;
    len += uint16_encode(p_cp_measurement->power_measurement, &p_encoded_buffer[len]);
		len = 20;
    // Flags Field
    p_encoded_buffer[0] = (uint8_t) ((flags >> 8) & 0x00FF);
    p_encoded_buffer[1] = (uint8_t) (flags & 0x00FF);

    return len;
}


/**@brief Function for adding cp Measurement characteristics.
 *
 * @param[in]   p_cps        Cycling Power Service structure.
 * @param[in]   p_cps_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cp_measurement_char_add(ble_cps_t * p_cps, const ble_cps_init_t * p_cps_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_cps_meas_t      initial_scm;
    uint8_t             encoded_scm[MAX_CPM_LEN];

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_cps_init->cp_meas_attr_md.cccd_write_perm;
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CYCLING_POWER_MEASUREMENT_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm );
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = cp_measurement_encode(p_cps, &initial_scm, encoded_scm);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MAX_CPM_LEN;
    attr_char_value.p_value      = encoded_scm;

    return sd_ble_gatts_characteristic_add(p_cps->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_cps->meas_handles);
}


/**@brief Function for adding cp Feature characteristics.
 *
 * @param[in]   p_cps        Cycling Power Service structure.
 * @param[in]   p_cps_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cp_feature_char_add(ble_cps_t * p_cps, const ble_cps_init_t * p_cps_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             init_value_encoded[4];
    uint8_t             init_value_len;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CYCLING_POWER_FEATURE_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cps_init->cp_feature_attr_md.read_perm;
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    init_value_len = uint32_encode(p_cps_init->feature, &init_value_encoded[0]);

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = init_value_len;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = init_value_len;
    attr_char_value.p_value      = init_value_encoded;

    return sd_ble_gatts_characteristic_add(p_cps->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_cps->feature_handles);
}


/**@brief Function for adding cp Sensor Location characteristic.
 *
 * @param[in]   p_cps        Cycling Power Service structure.
 * @param[in]   p_cps_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cp_sensor_loc_char_add(ble_cps_t * p_cps, const ble_cps_init_t * p_cps_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             init_value_len;
    uint8_t             encoded_init_value[1];

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_SENSOR_LOCATION_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cps_init->cp_sensor_loc_attr_md.read_perm;
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    init_value_len        = sizeof(uint8_t);
    if (p_cps_init->sensor_location != NULL)
    {
        encoded_init_value[0] = *p_cps_init->sensor_location;
    }

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = init_value_len;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = init_value_len;
    attr_char_value.p_value      = encoded_init_value;

    return sd_ble_gatts_characteristic_add(p_cps->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_cps->sensor_loc_handles);
}


uint32_t ble_cps_init(ble_cps_t * p_cps, const ble_cps_init_t * p_cps_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    //ble_cs_ctrlpt_init_t sc_ctrlpt_init;

    // Initialize service structure
    p_cps->evt_handler = p_cps_init->evt_handler;
    p_cps->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_cps->feature     = p_cps_init->feature;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CYCLING_POWER);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_cps->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Cycling Power measurement characteristic
    err_code = cp_measurement_char_add(p_cps, p_cps_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Cycling Power feature characteristic
    err_code = cp_feature_char_add(p_cps, p_cps_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Power Sensor Location characteristic
    if (p_cps_init->sensor_location != NULL)
    {
        err_code = cp_sensor_loc_char_add(p_cps, p_cps_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

		return err_code;
}


uint32_t ble_cps_measurement_send(ble_cps_t * p_cps, ble_cps_meas_t * p_measurement)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_cps->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_cp_meas[MAX_CPM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = cp_measurement_encode(p_cps, p_measurement, encoded_cp_meas);
        hvx_len = len;		// Length = 20 Bytes!!!!

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle   = p_cps->meas_handles.value_handle;
        hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset   = 0;
        hvx_params.p_len    = &hvx_len;
        hvx_params.p_data   = encoded_cp_meas;

        err_code = sd_ble_gatts_hvx(p_cps->conn_handle, &hvx_params);
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

uint32_t ble_cps_sensor_location_set(ble_cps_t * p_cps, uint8_t sensor_location)
{
    uint16_t len = sizeof(uint8_t);
    return sd_ble_gatts_value_set(p_cps->sensor_loc_handles.value_handle, 0, &len, &sensor_location);
}
