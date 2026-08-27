/**
 ******************************************************************************
 * @file    gw_debug.h
 * @brief   Live SWD window into the RUNNING gateway firmware.
 *
 * The gateway has no UART, and when the radio link is down it cannot tell you
 * anything over the air either - which is exactly when you most need to know
 * what it thinks is happening. The blink codes carry one number.
 *
 * This publishes the node's internal state into a single struct that OpenOCD
 * reads straight out of RAM through the ST-Link already attached:
 *
 *     cmake --build build --target read-gateway
 *
 * Unlike tests/nrf_probe, this is the REAL firmware. Nothing is reflashed and
 * nothing is stubbed - you are looking at the node that is actually failing,
 * with its actual radio and CAN state.
 *
 * COST: the read halts the core for a few milliseconds. That is harmless on a
 * bench, but do it with the car on blocks or disarmed - a halted gateway stops
 * publishing 0x200, and the vehicle node will (correctly) enter its CAN
 * timeout failsafe.
 ******************************************************************************
 */
#ifndef GW_DEBUG_H_
#define GW_DEBUG_H_

#include <stdint.h>
#include <stddef.h>

#define GW_DEBUG_MAGIC   0x47574442u   /* "GWDB" */

/**
 * @brief Everything worth knowing about a gateway that is not working.
 *
 * Flat, fixed layout, updated once per main-loop pass. `magic` is written last
 * so a reader can distinguish "filled in" from "uninitialised RAM", and `seq`
 * increments every pass so a stalled loop is obvious at a glance.
 */
typedef struct {
    uint32_t magic;
    uint32_t seq;            /**< main loop passes since reset               */
    uint32_t uptime_ms;

    /* ---- bring-up results ------------------------------------------- */
    uint8_t  clock_ok;
    uint8_t  can_ok;
    uint8_t  radio_ok;
    uint8_t  pad0;

    /* ---- radio, read back LIVE from the chip every pass -------------
     * The value of doing this here rather than in a probe: if the module
     * dies or a wire falls out while the node runs, these change and the
     * bring-up flags above do not. */
    uint8_t  nrf_config;     /**< 0x00 CONFIG                                */
    uint8_t  nrf_status;     /**< 0x07 STATUS                                */
    uint8_t  nrf_rf_ch;      /**< 0x05 RF_CH - expect CAR_RF_CHANNEL         */
    uint8_t  nrf_fifo;       /**< 0x17 FIFO_STATUS                           */

    /* ---- RF link ----------------------------------------------------- */
    uint32_t rf_frames;      /**< valid control frames received              */
    uint32_t rf_dropped;     /**< lost in transit, from the sequence counter */
    uint32_t rf_duplicates;
    uint32_t rf_age_ms;      /**< since the last valid frame                 */
    uint8_t  link_ok;
    uint8_t  link_quality;   /**< filled slots in the last 16                */
    uint8_t  pad1[2];

    /* ---- CAN --------------------------------------------------------- */
    uint32_t can_rx_status;  /**< 0x300 accepted                             */
    uint32_t can_rx_motion;  /**< 0x310 accepted                             */
    uint32_t can_tx_dropped; /**< no free mailbox                            */
    uint32_t e2e_errors;
    uint8_t  can_tec;
    uint8_t  can_rec;
    uint8_t  can_bus_off;
    uint8_t  pad2;

    /* ---- who is on the bus ------------------------------------------- */
    uint8_t  seen_vc;
    uint8_t  seen_sf;
    uint8_t  vc_state;       /**< VC_VehicleState from the last 0x300        */
    uint8_t  pad3;

    /* ---- sensor front object (0x100) ---------------------------------
     * APPEND ONLY BELOW THIS LINE. read_gateway.cmake decodes this struct by
     * word index, so inserting a field anywhere above silently shifts every
     * field after it and the script reports the wrong numbers with no error.
     *
     * These four answer the question the station's RANGE field cannot: when
     * it reads "---", is the sensor node silent, or is it talking and simply
     * seeing nothing? can_rx_object separates those immediately. */
    uint32_t can_rx_object;  /**< 0x100 accepted                             */
    uint16_t sf_range;       /**< SF_Range, mm, from the last 0x100          */
    uint8_t  sf_status;      /**< SF_Status: 0 none 1 valid 2 degraded 3 fault */
    uint8_t  sf_obj_fresh;   /**< 0x100 within SF_OBJECT_TIMEOUT_MS          */

    /* Times the link came back after having gone down. Separated from
     * rf_dropped because the 8-bit sequence wraps every ~5.1 s, so a gap
     * measured across a longer outage is meaningless - see sequence_track().
     *
     *   resyncs low, dropped low     healthy link
     *   resyncs > 0 only at start-up the station and car simply booted at
     *                                different moments. Expected, harmless.
     *   resyncs climbing while       the link is genuinely dropping out.
     *   driving                      THIS is the number that matters. */
    uint32_t link_resyncs;
} gw_debug_t;

/* The word indices tests/nrf_probe/read_gateway.cmake reads, asserted here so
 * a field inserted above breaks the BUILD rather than quietly making the
 * script print the wrong numbers against the right labels. If one of these
 * fires, either move your new field to the end of the struct, or update the
 * W(n) indices in that script to match. */
_Static_assert(offsetof(gw_debug_t, can_rx_status)  == 10u * 4u, "W10");
_Static_assert(offsetof(gw_debug_t, can_rx_motion)  == 11u * 4u, "W11");
_Static_assert(offsetof(gw_debug_t, can_tx_dropped) == 12u * 4u, "W12");
_Static_assert(offsetof(gw_debug_t, seen_vc)        == 15u * 4u, "W15");
_Static_assert(offsetof(gw_debug_t, can_rx_object)  == 16u * 4u, "W16");
_Static_assert(offsetof(gw_debug_t, sf_range)       == 17u * 4u, "W17");
_Static_assert(sizeof(gw_debug_t) == 19u * 4u, "mdw length in read_gateway.cmake");

extern volatile gw_debug_t g_gw_debug;

#endif /* GW_DEBUG_H_ */
