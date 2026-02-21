/**
 * @file ax25_physical.h
 * @brief AX.25 v2.2 Physical Layer State Machine and Interface
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header defines the physical layer interface for AX.25 v2.2 packet radio
 * communication. The physical layer manages transmitter control, CSMA/CA channel
 * access, frame queuing, and timing parameters required for reliable half-duplex
 * and full-duplex operation over radio channels.
 *
 * @section Standards_References
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2 (July 1998)
 *   https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * - KISS Protocol Specification (TAPR)
 *   https://www.ax25.net/kiss.aspx
 * - Extended sequence number (modulo-128) option for AX.25 (PE1CHL, DCC 1995)
 *   https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 *
 * @section Physical_Layer_Architecture
 * The AX.25 physical layer implements a state machine that controls transmitter
 * operation, manages channel access, and handles timing parameters. It operates
 * independently from the data link layer state machine defined in AX.25 v2.2
 * Section 2.8.
 *
 * @subsection Duplex_Modes
 * The physical layer supports two operational modes:
 * - Half-duplex: Shared medium operation with CSMA/CA, carrier sense, and
 *   transmitter hang timers to prevent channel monopolization
 * - Full-duplex: Dedicated TX/RX frequencies with simultaneous operation,
 *   bypassing CSMA mechanisms per AX.25 v2.2 XID negotiated parameters
 *
 * @section Timing_Parameters
 * All timing parameters are specified in 10ms units to allow 32-bit timestamps
 * to cover extended periods (up to ~248 days with wraparound handling).
 *
 * @subsection T100_AXHANG
 * T100 (axhang) is the transmitter hang timer that keeps PTT asserted after
 * the last frame transmission in half-duplex mode. This allows other stations
 * to detect channel occupancy and prevents premature channel seizure.
 * Default: 40 (400ms)
 *
 * @subsection T102_SLOTTIME
 * T102 (slottime) is the slot time timer for p-persistence CSMA algorithm.
 * When the channel clears, stations wait a random number of slot times before
 * transmitting to minimize collision probability.
 * Default: 0-10 (0-100ms), typical 100ms
 *
 * @subsection T103_TXDELAY
 * T103 (txdely) is the transmitter startup delay between PTT assertion and
 * actual data transmission. Allows transmitter stabilization and remote receiver
 * synchronization.
 * Default: 30 (300ms)
 *
 * @subsection T104_AXDELAY
 * T104 (axdelay) is the digipeater pre-transmit delay. Digipeated frames wait
 * this period after channel clear before transmitting to allow priority for
 * originated traffic.
 * Default: 60 (600ms)
 *
 * @subsection T105_REMOTE_SYNC
 * T105 (remote_sync) is the delay after PTT assertion before data transmission
 * to allow remote station receiver AGC and PLL lock-on. Only applicable in
 * half-duplex mode.
 * Default: 0 (disabled)
 *
 * @subsection T106_MAX_TX_DURATION
 * T106 (max_tx_duration) is the maximum continuous transmission duration to
 * prevent transmitter overheating and channel monopolization.
 * Default: 60000 (10 minutes)
 *
 * @subsection T107_ANTI_HOG
 * T107 (anti_hog) is the maximum session duration per burst. When exceeded,
 * the transmitter completes the current frame and releases PTT.
 * Default: 0 (disabled)
 *
 * @subsection T108_RX_STARTUP
 * T108 (rx_startup) is the receiver startup delay after transmitter unkey.
 * Ensures local receiver has stabilized before accepting incoming frames.
 * Default: 0 (disabled)
 *
 * @section CSMA_Algorithm
 * The physical layer implements p-persistent CSMA (Carrier Sense Multiple Access)
 * for half-duplex channel sharing:
 *
 * 1. Wait for channel clear (carrier_detect returns false)
 * 2. If DWAIT is configured, wait additional time since last foreign carrier
 * 3. Generate random number 0-255
 * 4. If random <= PERSIST, transmit immediately
 * 5. If random > PERSIST, wait one slot time and repeat
 * 6. After 16 deferred slots, force transmission
 *
 * @subsection DWAIT_Parameter
 * DWAIT provides additional politeness delay after detecting foreign carrier
 * to prevent collision with distant stations that may not have been heard.
 *
 * @section Frame_Queuing
 * The physical layer maintains a circular queue of frames awaiting transmission.
 * Frames are encoded AX.25 frames ready for HDLC framing (excluding flags and FCS).
 * The queue supports priority handling for digipeated frames.
 *
 * @section Receiver_State_Machine
 * Per AX.25 v2.2 Section 2.8, the duplex physical layer defines an independent
 * receiver state machine with two states:
 * - State 0 (READY): Receiver idle, ready to acquire frame synchronization
 * - State 1 (RECEIVING): Frame synchronization acquired, decoding in progress
 *
 * These states are managed separately from the transmitter state machine.
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef AX25_PHYSICAL_H_
#define AX25_PHYSICAL_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * @defgroup Physical_Constants Physical Layer Implementation Constants
 * @brief Size limits and buffer dimensions for physical layer operation
 *
 * These constants define the operational limits of the physical layer
 * implementation, including frame sizes, queue depths, and flag buffer
 * capacities.
 */

/**
 * @def AX25_PHYS_MAX_FRAME_SIZE
 * @brief Maximum frame size supported by the physical layer queue
 *
 * This value accommodates FX.25 frames with forward error correction
 * overhead (up to 255 bytes payload + RS redundancy) plus AX.25 headers.
 * Per AX.25 v2.2 Section 3.5, the standard maximum I-field is 256 octets,
 * but FX.25 extension may add up to 32 bytes of Reed-Solomon parity.
 *
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */
#define AX25_PHYS_MAX_FRAME_SIZE      512

/**
 * @def AX25_PHYS_QUEUE_SIZE
 * @brief Number of frames that can be queued for transmission
 *
 * Supports a full AX.25 window (k=7 for modulo-8, k=127 for modulo-128)
 * plus margin for supervisory frames and digipeated traffic.
 */
#define AX25_PHYS_QUEUE_SIZE          8

/**
 * @def AX25_PHYS_MAX_FLAGS
 * @brief Maximum number of flag bytes in preamble/interframe buffers
 *
 * Per AX.25 v2.2 Appendix C.2, preamble should be sufficient for remote
 * receiver synchronization (typically 20-40 flags at 1200bps).
 */
#define AX25_PHYS_MAX_FLAGS           64

/**
 * @defgroup Tick_Macros Tick Comparison Macros
 * @brief Wraparound-safe timestamp comparison utilities
 *
 * These macros handle 32-bit timestamp wraparound using signed arithmetic
 * on unsigned values. Correct for intervals up to ~248 days (2^31 * 10ms).
 */

/**
 * @def TICK_REACHED
 * @brief Check if current tick has reached or passed target tick
 *
 * Uses signed two's-complement subtraction to handle 32-bit wraparound.
 * Returns true when (now - target) >= 0 in signed arithmetic.
 *
 * @param now Current tick value in 10ms units
 * @param target Target tick value in 10ms units
 * @return Non-zero when target time has been reached or passed
 */
#define TICK_REACHED(now, target)  ((int32_t)((uint32_t)(now) - (uint32_t)(target)) >= 0)

/**
 * @def TICK_ELAPSED
 * @brief Calculate elapsed ticks between start and current time
 *
 * Returns signed elapsed time handling wraparound correctly.
 *
 * @param now Current tick value in 10ms units
 * @param start Start tick value in 10ms units
 * @return Signed elapsed ticks (may be negative if start is in future)
 */
#define TICK_ELAPSED(now, start)   ((int32_t)((uint32_t)(now) - (uint32_t)(start)))

/**
 * @defgroup Transmitter_States Transmitter State Machine States
 * @brief Physical layer transmitter state definitions
 *
 * The transmitter state machine controls the sequence of operations from
 * frame queueing through transmission completion. States progress from
 * IDLE through CSMA negotiation, keying delays, preamble transmission,
 * data transmission, and hang time before returning to IDLE.
 */

/**
 * @enum ax25_phys_state_t
 * @brief Transmitter state machine state enumeration
 *
 * Per AX.25 v2.2 Appendix C, the physical layer manages transmitter
 * timing and sequencing through these states:
 */
typedef enum {
    /**
     * @brief Idle state - no transmission in progress
     *
     * Transmitter is unkeyed. Frame queue may contain pending frames.
     * Transitions to CSMA_WAIT when frames are queued (half-duplex)
     * or directly to KEY_DELAY (full-duplex).
     */
    PHYS_IDLE,

    /**
     * @brief CSMA wait state - performing p-persistence algorithm
     *
     * Channel sense active. Waiting for channel clear, then performing
     * p-persistence random backoff to minimize collision probability.
     * Only entered in half-duplex mode.
     */
    PHYS_CSMA_WAIT,

    /**
     * @brief Key delay state - PTT asserted, waiting for transmitter ready
     *
     * Transmitter has been keyed (PTT asserted) and is waiting for
     * T103 (txdely) to elapse before transmitting preamble flags.
     */
    PHYS_KEY_DELAY,

    /**
     * @brief Remote sync state - allowing remote receiver synchronization
     *
     * Optional T105 delay after key delay to allow remote receiver AGC
     * and PLL to lock onto carrier before data transmission begins.
     */
    PHYS_REMOTE_SYNC,

    /**
     * @brief Preamble state - transmitting opening flag sequence
     *
     * Transmitting 0x7E flags to provide receiver synchronization
     * pattern before frame data. Per AX.25 v2.2 Section 3.11.
     */
    PHYS_PREAMBLE,

    /**
     * @brief Data state - transmitting frame content
     *
     * Active transmission of queued frame data via send_data callback.
     * T106 max duration timer is monitored during this state.
     */
    PHYS_DATA,

    /**
     * @brief Interframe state - between frames in a burst
     *
     * Transmitting inter-frame flags between back-to-back frames.
     * Allows frame boundary synchronization without unkeying.
     */
    PHYS_INTERFRAME,

    /**
     * @brief Hang state - transmission complete, maintaining PTT
     *
     * T100 (axhang) timer running to keep channel marked busy
     * after last frame. Prevents other stations from seizing
     * channel before acknowledgments can be sent.
     */
    PHYS_HANG
} ax25_phys_state_t;

/**
 * @defgroup Receiver_States Receiver State Machine States
 * @brief Physical layer receiver state definitions
 *
 * Per AX.25 v2.2 Section 2.8, the duplex physical layer defines an
 * independent two-state receiver state machine separate from the
 * transmitter states.
 */

/**
 * @enum ax25_phys_rx_state_t
 * @brief Receiver state machine state enumeration
 *
 * The receiver operates independently from the transmitter with two
 * distinct states as defined in AX.25 v2.2 Section 2.8:
 */
typedef enum {
    /**
     * @brief Receiver ready state (State 0)
     *
     * Receiver is idle and ready to acquire frame synchronization.
     * HDLC decoder is hunting for flag sequence (0x7E).
     */
    PHYS_RX_READY = 0,

    /**
     * @brief Receiver receiving state (State 1)
     *
     * Frame synchronization has been acquired (flag detected) and
     * HDLC decoder is processing frame content. Returns to READY
     * when frame completes (closing flag detected) or abort sequence
     * received.
     */
    PHYS_RX_RECEIVING
} ax25_phys_rx_state_t;

/**
 * @struct ax25_physical_t
 * @brief Physical layer context structure
 *
 * Contains all configuration parameters, state variables, frame queue,
 * and callback functions for the AX.25 physical layer implementation.
 * This structure should be initialized with ax25_physical_init() before
 * use and maintained for the duration of the link session.
 *
 * @section Configuration_Parameters
 * All timing parameters are in 10ms units unless otherwise noted.
 *
 * @subsection Duplex_Configuration
 * - full_duplex: Enables full-duplex operation bypassing CSMA
 *
 * @subsection Timing_Parameters
 * - txdely_10ms: T103 transmitter startup delay (default 30 = 300ms)
 * - axdelay_10ms: T104 digipeater delay (default 60 = 600ms)
 * - axhang_10ms: T100 transmitter hang time (default 40 = 400ms)
 * - slottime_10ms: T102 CSMA slot time (default 0-10 = 0-100ms)
 * - max_tx_duration_10ms: T106 maximum TX duration (default 60000 = 10min)
 * - dwait_10ms: Optional politeness delay after foreign carrier
 * - remote_sync_10ms: T105 remote receiver sync delay
 * - rx_startup_10ms: T108 receiver startup delay after TX
 * - anti_hog_10ms: T107 maximum burst duration limit
 *
 * @subsection CSMA_Parameters
 * - persist: P-persistence value 0-255 (default 63 ~ 0.25 probability)
 *
 * @subsection Framing_Parameters
 * - preamble_flags: Number of 0x7E flags before first frame (default 40)
 * - interframe_flags: Number of 0x7E flags between frames (default 5)
 *
 * @section Callback_Functions
 * The physical layer requires three mandatory callbacks:
 * - ptt_control: Controls transmitter PTT line
 * - carrier_detect: Returns channel busy status (CSMA input)
 * - send_data: Outputs frame bytes to modem/TNC
 *
 * Optional callback:
 * - abort_tx: Hardware abort for full-duplex recovery
 *
 * @section Frame_Queue
 * Circular queue holding up to AX25_PHYS_QUEUE_SIZE frames awaiting
 * transmission. Each slot stores:
 * - Frame data (up to AX25_PHYS_MAX_FRAME_SIZE bytes)
 * - Frame length
 * - Digipeat flag (for T104 delay handling)
 */
typedef struct {
    /**
     * @brief Full-duplex operation mode flag
     *
     * When true, CSMA is bypassed and half-duplex-only timers are
     * cleared. TX and RX operate simultaneously on separate frequencies.
     * Set via ax25_physical_set_duplex() to ensure proper state transition.
     */
    bool full_duplex;

    /**
     * @brief T103 - Transmitter startup delay in 10ms units
     *
     * Delay between PTT assertion and start of preamble transmission.
     * Allows transmitter power amplifier to stabilize and reach full
     * output power before modulation begins.
     * Default: 30 (300 milliseconds)
     */
    uint16_t txdely_10ms;

    /**
     * @brief T104 - Digipeater pre-transmit delay in 10ms units
     *
     * Additional delay applied to digipeated frames before PTT assertion.
     * Gives priority to originated traffic over repeated traffic.
     * Default: 60 (600 milliseconds)
     */
    uint16_t axdelay_10ms;

    /**
     * @brief T100 - Transmitter hang time in 10ms units
     *
     * Time PTT remains asserted after last frame transmission in
     * half-duplex mode. Keeps channel marked busy to prevent other
     * stations from transmitting before acknowledgments can be sent.
     * Default: 40 (400 milliseconds)
     */
    uint16_t axhang_10ms;

    /**
     * @brief P-persistence value for CSMA algorithm (0-255)
     *
     * Scaled probability of transmission after channel clear.
     * Value 255 = always transmit immediately (1-persistent).
     * Value 63 = ~25% probability (typical default).
     * Value 0 = defer always (not recommended).
     */
    uint8_t persist;

    /**
     * @brief T102 - CSMA slot time in 10ms units
     *
     * Time unit for p-persistence backoff. When channel clears and
     * persistence check fails, transmission is deferred by one slot
     * time before retry.
     * Default: 0-10 (0-100 milliseconds)
     */
    uint16_t slottime_10ms;

    /**
     * @brief T106 - Maximum transmission duration in 10ms units
     *
     * Maximum continuous transmission time to prevent channel
     * monopolization and transmitter overheating. When exceeded,
     * current frame completes with closing flags and PTT releases.
     * Default: 60000 (10 minutes)
     */
    uint32_t max_tx_duration_10ms;

    /**
     * @brief Optional DWAIT delay in 10ms units
     *
     * Additional politeness delay after foreign carrier detection.
     * Ensures distant stations not heard locally have cleared channel.
     * Default: 0 (disabled)
     */
    uint16_t dwait_10ms;

    /**
     * @brief T105 - Remote synchronization delay in 10ms units
     *
     * Delay after PTT assertion before data transmission to allow
     * remote receiver AGC and PLL to lock onto carrier.
     * Default: 0 (disabled)
     */
    uint16_t remote_sync_10ms;

    /**
     * @brief T108 - Receiver startup time in 10ms units
     *
     * Delay required after transmitter unkey before local receiver
     * is ready to receive. Prevents desensitization from local TX.
     * Default: 0 (disabled)
     */
    uint16_t rx_startup_10ms;

    /**
     * @brief T107 - Anti-hogging limit in 10ms units
     *
     * Maximum duration for a single transmission burst. When exceeded,
     * current frame completes and PTT releases to share channel.
     * Default: 0 (disabled)
     */
    uint16_t anti_hog_10ms;

    /**
     * @brief Number of preamble flags to transmit
     *
     * Count of 0x7E flag bytes sent before first frame in a burst.
     * Per AX.25 v2.2 Section 3.11, sufficient flags must be sent
     * to ensure remote receiver synchronization.
     * Default: 40 flags
     */
    uint8_t preamble_flags;

    /**
     * @brief Number of inter-frame flags to transmit
     *
     * Count of 0x7E flag bytes sent between back-to-back frames
     * in a burst. Provides frame boundary synchronization without
     * unkeying transmitter.
     * Default: 5 flags
     */
    uint8_t interframe_flags;

    /**
     * @brief PTT control callback function pointer
     *
     * Called to assert (on=true) or release (on=false) the transmitter
     * PTT (Push-To-Talk) line. Must be provided by hardware interface
     * layer.
     *
     * @param on True to key transmitter, false to unkey
     * @param user_data Opaque pointer passed through from user_data field
     */
    void (*ptt_control)(bool on, void *user_data);

    /**
     * @brief Carrier detect callback function pointer
     *
     * Called to determine if channel is busy (carrier present from
     * other stations). Required for CSMA operation in half-duplex
     * mode. May return false always in full-duplex mode.
     *
     * @param user_data Opaque pointer passed through from user_data field
     * @return True if channel busy (carrier detected), false if clear
     */
    bool (*carrier_detect)(void *user_data);

    /**
     * @brief Send data callback function pointer
     *
     * Called to output frame bytes to the modem or TNC for transmission.
     * Data is provided without HDLC flags or FCS - those are added
     * by the hardware layer if required.
     *
     * @param data Pointer to frame data bytes
     * @param len Length of data in bytes
     * @param user_data Opaque pointer passed through from user_data field
     */
    void (*send_data)(const uint8_t *data, size_t len, void *user_data);

    /**
     * @brief Abort transmission callback function pointer (optional)
     *
     * Called to abort in-progress transmission for full-duplex REJ
     * recovery. If NULL, abort is not supported by hardware.
     *
     * @param user_data Opaque pointer passed through from user_data field
     */
    void (*abort_tx)(void *user_data);

    /**
     * @brief Opaque user data pointer
     *
     * Passed to all callback functions to allow application context
     * to be associated with physical layer instance.
     */
    void *user_data;

    /**
     * @brief Frame queue storage buffer
     *
     * Circular queue holding up to AX25_PHYS_QUEUE_SIZE frames.
     * Each slot can hold one complete frame up to AX25_PHYS_MAX_FRAME_SIZE
     * bytes. Frames are stored as raw AX.25 frames (address + control +
     * info fields) ready for transmission.
     */
    uint8_t frame_storage[AX25_PHYS_QUEUE_SIZE][AX25_PHYS_MAX_FRAME_SIZE];

    /**
     * @brief Frame length array
     *
     * Parallel array to frame_storage indicating valid bytes in
     * each queue slot.
     */
    size_t frame_len[AX25_PHYS_QUEUE_SIZE];

    /**
     * @brief Digipeat flag array
     *
     * Parallel array to frame_storage indicating whether each queued
     * frame is a digipeated frame (true) or originated (false).
     * Digipeated frames are subject to T104 (axdelay) timing.
     */
    bool frame_is_digipeat[AX25_PHYS_QUEUE_SIZE];

    /**
     * @brief Queue head index
     *
     * Index of next frame to dequeue (oldest queued frame).
     * Queue is empty when queue_head == queue_tail.
     */
    uint8_t queue_head;

    /**
     * @brief Queue tail index
     *
     * Index where next frame will be enqueued (next free slot).
     * Queue is full when (queue_tail + 1) % AX25_PHYS_QUEUE_SIZE == queue_head.
     */
    uint8_t queue_tail;

    /**
     * @brief Current frame pointer
     *
     * Points to frame currently being transmitted (dequeued from queue).
     * NULL when no frame is actively being sent.
     */
    uint8_t *current_frame;

    /**
     * @brief Current frame length
     *
     * Length of frame pointed to by current_frame.
     */
    size_t current_len;

    /**
     * @brief Random number generator state
     *
     * LCG state for p-persistence algorithm. Initialized to deterministic
     * seed for testing; should be seeded with entropy from radio noise,
     * ADC, or timer jitter for operational use via ax25_physical_add_entropy().
     */
    uint32_t rng_state;

    /**
     * @brief Transmitter state machine state
     *
     * Current state of the transmitter state machine per ax25_phys_state_t.
     */
    ax25_phys_state_t state;

    /**
     * @brief Receiver state machine state
     *
     * Current state of the independent receiver state machine per
     * AX.25 v2.2 Section 2.8.
     */
    ax25_phys_rx_state_t rx_state;

    /**
     * @brief Next action timestamp in 10ms units
     *
     * Target tick for next state transition or timer expiry.
     * Compared against current tick using TICK_REACHED macro.
     */
    uint32_t next_action_tick_10ms;

    /**
     * @brief Transmission start timestamp in 10ms units
     *
     * Tick when current transmission burst began (PTT asserted).
     * Used for T106 max duration enforcement.
     */
    uint32_t tx_start_tick_10ms;

    /**
     * @brief Transmitter active flag
     *
     * True when PTT is asserted and transmitter is keyed.
     * Set by ptt_control(true) and cleared by ptt_control(false).
     */
    bool tx_active;

    /**
     * @brief Flag buffer for preamble/interframe transmission
     *
     * Pre-filled buffer of 0x7E flag bytes for efficient transmission
     * via send_data callback.
     */
    uint8_t flag_buf[AX25_PHYS_MAX_FLAGS];

    /**
     * @brief Persistence deferral flag
     *
     * True when p-persistence check has failed and station is
     * deferring transmission for slottime before retry.
     */
    bool persistence_deferred;

    /**
     * @brief Persistence slot counter
     *
     * Count of consecutive deferred slots. After 16 deferrals,
     * transmission is forced regardless of persistence check.
     */
    uint8_t persistence_slots_remaining;

    /**
     * @brief Timestamp of last foreign carrier detection
     *
     * Tick when carrier_detect last returned true (channel busy).
     * Used for DWAIT calculation.
     */
    uint32_t last_foreign_carrier_tick_10ms;

    /**
     * @brief DWAIT pending flag
     *
     * True when DWAIT period must be observed after channel clear
     * due to recent foreign carrier detection.
     */
    bool dwait_pending;

    /**
     * @brief Last transmitter unkey timestamp
     *
     * Tick when PTT was last released. Used for T108 receiver
     * startup delay calculation.
     */
    uint32_t last_unkey_tick_10ms;

    /**
     * @brief Receiver warmup required flag
     *
     * True when T108 receiver startup delay must be observed
     * before next transmission can begin.
     */
    bool rx_warmup_required;

    /**
     * @brief Anti-hog timer expired flag
     *
     * True when T107 anti-hog limit has been exceeded. Causes
     * transmitter to release PTT after current frame completion.
     */
    bool anti_hog_expired;

    /**
     * @brief Current session start timestamp
     *
     * Tick when current transmission session began. Used for
     * T107 anti-hog timer calculation.
     */
    uint32_t current_session_start_10ms;

    /**
     * @brief AXDELAY pending flag
     *
     * True when T104 digipeater delay is being enforced before
     * PTT assertion for a digipeated frame.
     */
    bool axdelay_pending;

    /**
     * @brief Last processed tick timestamp
     *
     * Last tick value passed to ax25_physical_tick(). Used for
     * internal kickstart timing calculations.
     */
    uint32_t last_tick_10ms;

    /**
     * @brief T106 inter-burst pause pending flag
     *
     * True when T106 has fired in full-duplex mode and mandatory
     * inter-burst pause (txdely) is being enforced before re-keying.
     */
    bool t106_inter_burst_pending;
} ax25_physical_t;

/**
 * @defgroup Initialization_Functions Initialization and Configuration
 * @brief Physical layer setup and configuration functions
 */

/**
 * @brief Initialize physical layer context with default parameters
 *
 * Initializes all fields of the physical layer structure to default
 * values per AX.25 v2.2 recommendations. Must be called before any
 * other physical layer functions.
 *
 * @section Default_Values
 * - txdely_10ms: 30 (300ms)
 * - axdelay_10ms: 60 (600ms)
 * - axhang_10ms: 40 (400ms)
 * - persist: 63 (~25%)
 * - slottime_10ms: 0 (immediate)
 * - max_tx_duration_10ms: 60000 (10 minutes)
 * - preamble_flags: 40
 * - interframe_flags: 5
 * - rng_state: 0x12345678 (deterministic seed)
 * - state: PHYS_IDLE
 * - rx_state: PHYS_RX_READY
 *
 * @param phys Pointer to physical layer structure to initialize
 *
 * @note Callback functions (ptt_control, carrier_detect, send_data)
 *       must be assigned by caller after initialization.
 * @note For operational use, call ax25_physical_add_entropy() with
 *       hardware-derived noise to seed the RNG unpredictably.
 */
void ax25_physical_init(ax25_physical_t *phys);

/**
 * @brief Configure full-duplex or half-duplex operation mode
 *
 * Sets the duplex mode and adjusts timing parameters accordingly.
 * In full-duplex mode, CSMA is bypassed and half-duplex-only timers
 * are cleared as they are meaningless on dedicated TX/RX frequencies.
 *
 * @section Full_Duplex_Changes
 * - full_duplex set to true
 * - dwait_10ms cleared to 0
 * - axhang_10ms cleared to 0
 * - anti_hog_10ms cleared to 0
 * - remote_sync_10ms cleared to 0
 * - rx_startup_10ms cleared to 0
 * - persist set to 255 (immediate transmission)
 * - slottime_10ms cleared to 0
 * - axdelay_10ms cleared to 0
 *
 * @param phys Pointer to physical layer structure
 * @param fd True for full-duplex, false for half-duplex
 *
 * @note Should be called after ax25_physical_init() and after XID
 *       parameter negotiation has established duplex capability.
 * @note Changing duplex mode during active transmission may result
 *       in undefined behavior.
 */
void ax25_physical_set_duplex(ax25_physical_t *phys, bool fd);

/**
 * @defgroup Queue_Management Frame Queue Management
 * @brief Functions for managing the transmission frame queue
 */

/**
 * @brief Queue a frame for transmission
 *
 * Adds an encoded AX.25 frame to the transmission queue. The frame
 * will be transmitted according to current physical layer state and
 * timing parameters. In half-duplex mode, may trigger immediate
 * processing via ax25_physical_tick() if state is IDLE.
 *
 * @section Queue_Behavior
 * - Frames are queued FIFO except digipeated frames get T104 delay
 * - Queue full condition returns false (frame must be retried later)
 * - Frame data is copied to internal buffer (caller may free after return)
 * - Maximum frame size is AX25_PHYS_MAX_FRAME_SIZE bytes
 *
 * @param phys Pointer to physical layer structure
 * @param frame Pointer to encoded AX.25 frame data
 * @param len Length of frame data in bytes (must be > 0)
 * @param is_digipeat True if frame is digipeated (subject to T104 delay)
 *
 * @return True if frame queued successfully, false if queue full or
 *         parameters invalid
 *
 * @note Frame should be complete AX.25 frame (address + control + info)
 *       excluding HDLC flags and FCS which are added by hardware layer.
 */
bool ax25_physical_queue_frame(ax25_physical_t *phys, const uint8_t *frame, size_t len, bool is_digipeat);

/**
 * @defgroup Entropy_Management Random Number Generation
 * @brief Functions for seeding the p-persistence RNG
 */

/**
 * @brief Add entropy to the random number generator
 *
 * Mixes external entropy into the LCG random number generator state
 * used for p-persistence CSMA. Should be called with hardware-derived
 * noise (radio RSSI, ADC samples, timer jitter) to prevent predictable
 * transmission timing patterns.
 *
 * @section Entropy_Sources
 * Suitable entropy sources include:
 * - Radio receiver noise floor (RSSI) samples
 * - Analog-to-digital converter (ADC) readings of unconnected pins
 * - High-resolution timer jitter measurements
 * - Temperature sensor noise
 *
 * @param phys Pointer to physical layer structure
 * @param noise_sample 8-bit entropy sample to mix into RNG state
 *
 * @note The default seed (0x12345678) provides deterministic behavior
 *       suitable for testing but should be reseeded for operational use.
 * @note May be called from interrupt context; operation is atomic.
 */
void ax25_physical_add_entropy(ax25_physical_t *phys, uint8_t noise_sample);

/**
 * @defgroup State_Machine Tick Processing and State Machine
 * @brief Main physical layer processing function
 */

/**
 * @brief Process physical layer state machine for one time tick
 *
 * Main state machine processing function that must be called
 * periodically (recommended every 10ms) to advance transmitter
 * states, manage timers, and handle frame transmission sequencing.
 *
 * @section Timing_Requirements
 * - Should be called at regular intervals (10ms units)
 * - tick_10ms should increment by 1 per call for real-time operation
 * - May be called with arbitrary increments for simulation/testing
 * - Wraparound of tick_10ms is handled correctly via TICK_REACHED
 *
 * @section State_Processing
 * Processes state transitions for:
 * - PHYS_IDLE: Check queue, initiate CSMA or keying
 * - PHYS_CSMA_WAIT: P-persistence algorithm and carrier sense
 * - PHYS_KEY_DELAY: T103 transmitter startup delay
 * - PHYS_REMOTE_SYNC: T105 remote receiver synchronization
 * - PHYS_PREAMBLE: Transmit opening flag sequence
 * - PHYS_DATA: Transmit frame data, monitor T106
 * - PHYS_INTERFRAME: Transmit inter-frame flags
 * - PHYS_HANG: T100 hang time before unkey
 *
 * @param phys Pointer to physical layer structure
 * @param tick_10ms Current time in 10ms units (monotonic)
 *
 * @note This function drives all physical layer timing and must be
 *       called regularly for proper operation.
 * @note In full-duplex mode, CSMA states are bypassed.
 */
void ax25_physical_tick(ax25_physical_t *phys, uint32_t tick_10ms);

/**
 * @defgroup Recovery_Functions Error Recovery Functions
 * @brief Functions for handling transmission errors and recovery
 */

/**
 * @brief Abort the currently transmitting frame
 *
 * Immediately aborts in-progress transmission for full-duplex REJ
 * recovery. Flushes the transmit queue and clears current frame
 * pointer while keeping PTT asserted so retransmissions can be
 * queued and sent immediately.
 *
 * @section Abort_Sequence
 * 1. Calls abort_tx callback if provided (hardware-level abort)
 * 2. Flushes all queued frames (queue_head = queue_tail)
 * 3. Clears current_frame pointer
 * 4. PTT remains ON for immediate retransmission
 *
 * @param phys Pointer to physical layer structure
 *
 * @note Only effective when state is PHYS_DATA or PHYS_INTERFRAME
 * @note No-op if abort_tx callback is NULL (software abort only)
 * @note Intended for full-duplex mode where immediate recovery is needed
 */
void ax25_physical_abort_current_frame(ax25_physical_t *phys);

/**
 * @defgroup Receiver_Functions Receiver State Management
 * @brief Functions for managing the independent receiver state machine
 */

/**
 * @brief Signal frame reception start to receiver state machine
 *
 * Called by the hardware HDLC decoder when frame synchronization
 * is acquired (opening flag detected). Transitions receiver state
 * from PHYS_RX_READY to PHYS_RX_RECEIVING per AX.25 v2.2 Section 2.8.
 *
 * @section Usage
 * This function should be called from the HDLC decoder interrupt
 * handler or event loop when a flag sequence (0x7E) is detected
 * indicating start of frame reception.
 *
 * @param phys Pointer to physical layer structure
 *
 * @note Safe to call from interrupt context; touches only rx_state
 * @note Transmitter state machine is unaffected by this call
 * @note After calling, application should prepare to receive frame data
 */
void ax25_physical_rx_frame_start(ax25_physical_t *phys);

/**
 * @brief Signal frame reception completion to receiver state machine
 *
 * Called by the hardware HDLC decoder when frame reception completes
 * (closing flag detected or abort sequence received). Transitions
 * receiver state from PHYS_RX_RECEIVING to PHYS_RX_READY.
 *
 * @section Usage
 * This function should be called after a complete frame has been
 * received and validated (FCS checked). The application should then
 * process the received frame via ax25_process_frame() or equivalent.
 *
 * @param phys Pointer to physical layer structure
 *
 * @note Safe to call from interrupt context; touches only rx_state
 * @note Transmitter state machine is unaffected by this call
 * @note Receiver is immediately ready to acquire next frame synchronization
 */
void ax25_physical_rx_frame_end(ax25_physical_t *phys);

#endif /* AX25_PHYSICAL_H_ */
