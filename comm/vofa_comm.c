#include "comm/vofa_comm.h"
#include "control/ControlLoop.h"
#include "board/board_sci.h"

#define VOFA_COMM_FRAME_MAX_SIZE \
    ((VOFA_COMM_CHANNEL_COUNT * 4U) + VOFA_COMM_TAIL_SIZE)

#define VOFA_RX_WAIT_HEAD1       0U
#define VOFA_RX_WAIT_HEAD2       1U
#define VOFA_RX_WAIT_CMD1        2U
#define VOFA_RX_WAIT_CMD2        3U
#define VOFA_RX_WAIT_VALUE       4U
#define VOFA_RX_WAIT_TAIL2       5U

#define VOFA_CMD_ACTION_VPI      0U
#define VOFA_CMD_ACTION_IPI      1U
#define VOFA_CMD_ACTION_TARGET   2U

extern volatile float32_t gControlVpiKp;
extern volatile float32_t gControlVpiKi;
extern volatile float32_t gControlIpiKp;
extern volatile float32_t gControlIpiKi;
extern volatile float32_t TempTargetVoltage;
extern volatile float32_t TempTargetCurrent;
extern CTRLCSS DABCSS;

typedef struct
{
    char cmd0;
    char cmd1;
    volatile float32_t *target;
    uint16_t action;
} VOFA_CmdMap_t;

static volatile float gVofaChannels[VOFA_COMM_CHANNEL_COUNT];
static volatile bool gVofaFramePending;

static uint16_t gVofaTxBuffer[VOFA_COMM_FRAME_MAX_SIZE];
static uint16_t gVofaTxLen;
static uint16_t gVofaTxIndex;
static char gVofaRxCmd[2];
static char gVofaRxValue[VOFA_COMM_RX_VALUE_MAX_LEN + 1U];
static uint16_t gVofaRxState = VOFA_RX_WAIT_HEAD1;
static uint16_t gVofaRxValueLen;

static const uint16_t gVofaJustFloatTail[VOFA_COMM_TAIL_SIZE] =
{
    0x00U, 0x00U, 0x80U, 0x7FU
};

static void VOFA_CommPackFloat(float value, uint16_t *buf)
{
    union
    {
        float f32;
        uint32_t u32;
    } data;

    data.f32 = value;

    buf[0] = (uint16_t)(data.u32 & 0xFFUL);
    buf[1] = (uint16_t)((data.u32 >> 8U) & 0xFFUL);
    buf[2] = (uint16_t)((data.u32 >> 16U) & 0xFFUL);
    buf[3] = (uint16_t)((data.u32 >> 24U) & 0xFFUL);
}

static const VOFA_CmdMap_t gVofaCmdMap[] =
{
    {'V', 'P', &gControlVpiKp,     VOFA_CMD_ACTION_VPI},
    {'V', 'I', &gControlVpiKi,     VOFA_CMD_ACTION_VPI},
    {'I', 'P', &gControlIpiKp,     VOFA_CMD_ACTION_IPI},
    {'I', 'I', &gControlIpiKi,     VOFA_CMD_ACTION_IPI},
    {'V', 'T', &TempTargetVoltage, VOFA_CMD_ACTION_TARGET},
    {'I', 'T', &TempTargetCurrent, VOFA_CMD_ACTION_TARGET}
};

static void VOFA_CommResetRx(void)
{
    gVofaRxState = VOFA_RX_WAIT_HEAD1;
    gVofaRxValueLen = 0U;
}

static bool VOFA_CommParseFloat(const char *str, float32_t *value)
{
    float32_t sign = 1.0f;
    float32_t result = 0.0f;
    float32_t frac = 0.1f;
    bool hasDigit = false;

    if ((str == (const char *)0) || (value == (float32_t *)0))
    {
        return false;
    }

    if (*str == '-')
    {
        sign = -1.0f;
        str++;
    }
    else if (*str == '+')
    {
        str++;
    }

    while ((*str >= '0') && (*str <= '9'))
    {
        hasDigit = true;
        result = (result * 10.0f) + (float32_t)(*str - '0');
        str++;
    }

    if (*str == '.')
    {
        str++;
        while ((*str >= '0') && (*str <= '9'))
        {
            hasDigit = true;
            result += ((float32_t)(*str - '0') * frac);
            frac *= 0.1f;
            str++;
        }
    }

    if ((*str != '\0') || (!hasDigit))
    {
        return false;
    }

    *value = result * sign;
    return true;
}

static void VOFA_CommApplyCommand(void)
{
    float32_t value;
    uint16_t i;

    gVofaRxValue[gVofaRxValueLen] = '\0';

    if ((!VOFA_CommParseFloat(gVofaRxValue, &value)) || (value < 0.0f))
    {
        return;
    }

    for (i = 0U; i < (sizeof(gVofaCmdMap) / sizeof(gVofaCmdMap[0])); i++)
    {
        if ((gVofaRxCmd[0] == gVofaCmdMap[i].cmd0) &&
            (gVofaRxCmd[1] == gVofaCmdMap[i].cmd1))
        {
            *(gVofaCmdMap[i].target) = value;

            if (gVofaCmdMap[i].action == VOFA_CMD_ACTION_VPI)
            {
                ControlLoop_requestVoltagePIUpdate();
            }
            else if (gVofaCmdMap[i].action == VOFA_CMD_ACTION_IPI)
            {
                ControlLoop_requestCurrentPIUpdate();
            }
            else if (DABCSS.sts != DABERROR)
            {
                DABCSS.sts = DABWAITCHANGE;
            }

            return;
        }
    }
}

/* Initialize VOFA communication cache.
 * 初始化 VOFA 通讯缓存。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 */
void VOFA_CommInit(void)
{
    uint16_t i;

    for (i = 0U; i < VOFA_COMM_CHANNEL_COUNT; i++)
    {
        gVofaChannels[i] = 0.0f;
    }

    gVofaFramePending = false;
    gVofaTxLen = 0U;
    gVofaTxIndex = 0U;
    VOFA_CommResetRx();
}

/* Set one VOFA channel value.
 * 设置一个 VOFA 通道值。
 *
 * Input:
 *   channel - Channel index.
 *             通道下标。
 *   value   - Floating point value to send.
 *             要发送的浮点值。
 * Output:
 *   None.
 */
void VOFA_CommSetChannel(uint16_t channel, float value)
{
    if (channel < VOFA_COMM_CHANNEL_COUNT)
    {
        gVofaChannels[channel] = value;
    }
}

/* Copy a group of channel values into VOFA cache.
 * 将一组通道值复制到 VOFA 缓存。
 *
 * Input:
 *   data  - Pointer to float channel data.
 *           float 通道数据指针。
 *   count - Number of channels to copy.
 *           需要复制的通道数量。
 * Output:
 *   None.
 */
void VOFA_CommSetChannels(const float *data, uint16_t count)
{
    uint16_t i;
    uint16_t copyCount;

    if (data == (const float *)0)
    {
        return;
    }

    copyCount = (count < VOFA_COMM_CHANNEL_COUNT) ? count : VOFA_COMM_CHANNEL_COUNT;

    for (i = 0U; i < copyCount; i++)
    {
        gVofaChannels[i] = data[i];
    }
}

/* Mark a frame request from control ISR.
 * 在控制中断中置位一帧发送请求。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 */
void VOFA_CommMarkFrameFromISR(void)
{
    gVofaFramePending = true;
}

/* VOFA background communication task.
 * VOFA 后台通讯任务。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 */
void VOFA_CommTask(void)
{
    float data[VOFA_COMM_CHANNEL_COUNT];
    uint16_t written;
    uint16_t i;

    if (gVofaTxIndex < gVofaTxLen)
    {
        written = VOFA_CommWriteBytes(&gVofaTxBuffer[gVofaTxIndex],
                                      (uint16_t)(gVofaTxLen - gVofaTxIndex));
        gVofaTxIndex += written;

        if (gVofaTxIndex < gVofaTxLen)
        {
            return;
        }

        gVofaTxLen = 0U;
        gVofaTxIndex = 0U;
    }

    if (!gVofaFramePending)
    {
        return;
    }

    gVofaFramePending = false;

    for (i = 0U; i < VOFA_COMM_CHANNEL_COUNT; i++)
    {
        data[i] = gVofaChannels[i];
    }

    (void)VOFA_CommSendFloatFrame(data, VOFA_COMM_CHANNEL_COUNT);
}

/* Feed one received character into VOFA command parser.
 * 向 VOFA 命令解析器喂入一个接收到的字符。
 *
 * Input:
 *   ch - One ASCII character received from SCI/USB.
 *        从 SCI/USB 接收到的一个 ASCII 字符。
 * Output:
 *   None.
 */
void VOFA_CommRxChar(char ch)
{
    switch (gVofaRxState)
    {
        case VOFA_RX_WAIT_HEAD1:
            if (ch == 'A')
            {
                gVofaRxState = VOFA_RX_WAIT_HEAD2;
            }
            break;

        case VOFA_RX_WAIT_HEAD2:
            gVofaRxState = (ch == 'A') ? VOFA_RX_WAIT_CMD1 : VOFA_RX_WAIT_HEAD1;
            break;

        case VOFA_RX_WAIT_CMD1:
            gVofaRxCmd[0] = ch;
            gVofaRxState = VOFA_RX_WAIT_CMD2;
            break;

        case VOFA_RX_WAIT_CMD2:
            gVofaRxCmd[1] = ch;
            gVofaRxValueLen = 0U;
            gVofaRxState = VOFA_RX_WAIT_VALUE;
            break;

        case VOFA_RX_WAIT_VALUE:
            if (ch == 'B')
            {
                gVofaRxState = VOFA_RX_WAIT_TAIL2;
            }
            else if (gVofaRxValueLen < VOFA_COMM_RX_VALUE_MAX_LEN)
            {
                gVofaRxValue[gVofaRxValueLen] = ch;
                gVofaRxValueLen++;
            }
            else
            {
                VOFA_CommResetRx();
            }
            break;

        case VOFA_RX_WAIT_TAIL2:
            if (ch == 'B')
            {
                VOFA_CommApplyCommand();
            }
            VOFA_CommResetRx();
            break;

        default:
            VOFA_CommResetRx();
            break;
    }
}

/* Feed a received byte array into VOFA command parser.
 * 向 VOFA 命令解析器喂入一组接收字节。
 *
 * Input:
 *   buf - Received byte buffer.
 *         接收字节缓冲区。
 *   len - Received byte length.
 *         接收字节长度。
 * Output:
 *   None.
 */
void VOFA_CommRxBytes(const uint16_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == (const uint16_t *)0)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        VOFA_CommRxChar((char)buf[i]);
    }
}

/* Poll SCIB RX FIFO and feed received bytes into VOFA parser.
 * 轮询 SCIB RX FIFO，并把收到的字节送入 VOFA 解析器。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 */
void VOFA_CommRxTask(void)
{
    uint16_t ch;

    while (SCI_getRxFIFOStatus(BOARD_SCIB_BASE) != SCI_FIFO_RX0)
    {
        ch = SCI_readCharNonBlocking(BOARD_SCIB_BASE);
        VOFA_CommRxChar((char)(ch & 0x00FFU));
    }
}

/* Send one JustFloat frame immediately.
 * 立即发送一帧 JustFloat 数据。
 *
 * Input:
 *   data  - Pointer to float channel data.
 *           float 通道数据指针。
 *   count - Number of float channels.
 *           float 通道数量。
 * Output:
 *   Number of bytes accepted by the low-level port.
 *   底层端口接受的字节数。
 */
uint16_t VOFA_CommSendFloatFrame(const float *data, uint16_t count)
{
    uint16_t sendCount;
    uint16_t dataBytes;
    uint16_t frameBytes;
    uint16_t written;
    uint16_t i;

    if (data == (const float *)0)
    {
        return 0U;
    }

    if (gVofaTxIndex < gVofaTxLen)
    {
        return 0U;
    }

    sendCount = (count < VOFA_COMM_CHANNEL_COUNT) ? count : VOFA_COMM_CHANNEL_COUNT;
    dataBytes = (uint16_t)(sendCount * 4U);
    frameBytes = (uint16_t)(dataBytes + VOFA_COMM_TAIL_SIZE);

    for (i = 0U; i < sendCount; i++)
    {
        VOFA_CommPackFloat(data[i], &gVofaTxBuffer[i * 4U]);
    }

    for (i = 0U; i < VOFA_COMM_TAIL_SIZE; i++)
    {
        gVofaTxBuffer[dataBytes + i] = gVofaJustFloatTail[i];
    }

    gVofaTxLen = frameBytes;
    gVofaTxIndex = 0U;

    written = VOFA_CommWriteBytes(gVofaTxBuffer, gVofaTxLen);
    gVofaTxIndex = written;

    if (gVofaTxIndex >= gVofaTxLen)
    {
        gVofaTxLen = 0U;
        gVofaTxIndex = 0U;
    }

    return written;
}

/* Low-level byte write hook.
 * 底层字节发送接口。
 *
 * Input:
 *   buf - Byte buffer to send.
 *         待发送字节缓冲区。
 *   len - Byte length.
 *         字节长度。
 * Output:
 *   Number of bytes accepted by the low-level port.
 *   底层端口接受的字节数。
 */
uint16_t VOFA_CommWriteBytes(const uint16_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == (const uint16_t *)0)
    {
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        if (SCI_getTxFIFOStatus(BOARD_SCIB_BASE) == SCI_FIFO_TX16)
        {
            break;
        }

        SCI_writeCharNonBlocking(BOARD_SCIB_BASE, (uint16_t)buf[i]);
    }

    return i;
}
