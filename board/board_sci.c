#include "board/board_sci.h"

/* Configure one SCI pin.
 * 配置一个 SCI 引脚。
 *
 * Input:
 *   gpio      - GPIO number.
 *               GPIO 编号。
 *   pinConfig - Driverlib pinmux value, such as GPIO_9_SCIB_TX.
 *               Driverlib PinMux 配置值，例如 GPIO_9_SCIB_TX。
 *   direction - GPIO direction used by the SCI pin.
 *               SCI 引脚方向。
 * Output:
 *   None.
 */
#define BOARD_CONFIG_SCI_PIN(gpio, pinConfig, direction)       \
    do                                                        \
    {                                                         \
        GPIO_setControllerCore((gpio), GPIO_CORE_CPU1);       \
        GPIO_setPinConfig((pinConfig));                       \
        GPIO_setDirectionMode((gpio), (direction));           \
        GPIO_setPadConfig((gpio), GPIO_PIN_TYPE_STD);         \
        GPIO_setQualificationMode((gpio), GPIO_QUAL_ASYNC);   \
    } while (0)

/* Initialize SCIB GPIO pinmux for the CH340 debug UART.
 * 初始化 CH340 调试串口使用的 SCIB GPIO 复用。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 */
void Board_SCIB_initGPIO(void)
{
    /* SCIB_TX: GPIO9 -> CH340 RXD.
     * SCIB_TX：GPIO9 接 CH340 RXD。 */
    BOARD_CONFIG_SCI_PIN(BOARD_SCIB_TX_GPIO,
                         BOARD_SCIB_TX_PIN_CONFIG,
                         GPIO_DIR_MODE_OUT);

    /* SCIB_RX: GPIO11 <- CH340 TXD.
     * SCIB_RX：GPIO11 接 CH340 TXD。 */
    BOARD_CONFIG_SCI_PIN(BOARD_SCIB_RX_GPIO,
                         BOARD_SCIB_RX_PIN_CONFIG,
                         GPIO_DIR_MODE_IN);
}

/* Initialize SCIB as an 8N1 FIFO UART.
 * 将 SCIB 初始化为 8N1 FIFO 串口。
 *
 * Input:
 *   baudRate - UART baud rate.
 *              串口波特率。
 * Output:
 *   None.
 */
void Board_SCIB_init(uint32_t baudRate)
{
    Board_SCIB_initGPIO();

    /* Make sure the SCIB peripheral clock is enabled.
     * 确保 SCIB 外设时钟已经打开。 */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIB);

    /* Clear SCI internal state before the baud rate and frame format are updated.
     * 配置波特率和帧格式前，先清一次 SCI 内部状态。 */
    SCI_performSoftwareReset(BOARD_SCIB_BASE);

    /* 8 data bits, 1 stop bit, no parity.
     * 8 位数据位，1 位停止位，无校验。 */
    SCI_setConfig(BOARD_SCIB_BASE,
                  DEVICE_LSPCLK_FREQ,
                  baudRate,
                  (SCI_CONFIG_WLEN_8 |
                   SCI_CONFIG_STOP_ONE |
                   SCI_CONFIG_PAR_NONE));

    SCI_resetChannels(BOARD_SCIB_BASE);
    SCI_resetRxFIFO(BOARD_SCIB_BASE);
    SCI_resetTxFIFO(BOARD_SCIB_BASE);
    SCI_clearInterruptStatus(BOARD_SCIB_BASE, SCI_INT_TXFF | SCI_INT_RXFF);

    /* FIFO is enabled, but interrupts are intentionally kept disabled.
     * 打开 FIFO，但这里故意不使能中断，避免影响控制中断链路。 */
    SCI_enableFIFO(BOARD_SCIB_BASE);
    SCI_enableModule(BOARD_SCIB_BASE);
    SCI_performSoftwareReset(BOARD_SCIB_BASE);
}
