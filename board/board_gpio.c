#include "board.h"

#define DSP_PWM9A_GPIO      16U
#define DSP_PWM9A_GPIOMUX   GPIO_16_EPWM9A
#define DSP_PWM9B_GPIO      17U
#define DSP_PWM9B_GPIOMUX   GPIO_17_EPWM9B

#define DSP_PWM10A_GPIO     18U
#define DSP_PWM10A_GPIOMUX  GPIO_18_EPWM10A
#define DSP_PWM10B_GPIO     19U
#define DSP_PWM10B_GPIOMUX  GPIO_19_EPWM10B

#define DSP_PWM11A_GPIO     20U
#define DSP_PWM11A_GPIOMUX  GPIO_20_EPWM11A
#define DSP_PWM11B_GPIO     21U
#define DSP_PWM11B_GPIOMUX  GPIO_21_EPWM11B

#define DSP_PWM12A_GPIO     22U
#define DSP_PWM12A_GPIOMUX  GPIO_22_EPWM12A
#define DSP_PWM12B_GPIO     23U
#define DSP_PWM12B_GPIOMUX  GPIO_23_EPWM12B

/* Configure one EPWM pinmux pair.
 * 配置一个 EPWM 引脚：这里只做 PinMux、Pad、Qualification 和 CPU1 归属。
 *
 * EPWM 引脚不是普通 GPIO 输出，不在这里调用 GPIO_setDirectionMode()
 * 或 GPIO_writePin()。波形由 EPWM 外设输出。
 */
#define BOARD_CONFIG_EPWM_PIN(gpio, mux)        \
    do                                          \
    {                                           \
        GPIO_setPinConfig((mux));               \
        GPIO_setPadConfig((gpio), GPIO_PIN_TYPE_STD); \
        GPIO_setQualificationMode((gpio), GPIO_QUAL_SYNC); \
        GPIO_setControllerCore((gpio), GPIO_CORE_CPU1); \
    } while (0)

/* GPIO initialization.
 * GPIO 初始化。
 *
 * 当前按 F28388D 常见映射打开控制链使用的 PWM9~PWM12 A/B PinMux：
 *   EPWM9A/B  -> GPIO16/17
 *   EPWM10A/B -> GPIO18/19
 *   EPWM11A/B -> GPIO20/21
 *   EPWM12A/B -> GPIO22/23
 *
 * 如果原理图没有使用 GPIO16~GPIO23 这组默认 PWM 引脚，需要在上面的
 * DSP_PWMxA/B_GPIO 和 DSP_PWMxA/B_GPIOMUX 宏里按实际引脚修改。
 */
void Board_initGPIO(void)
{
    /* PWM9 pins / PWM9 引脚初始化 */
    BOARD_CONFIG_EPWM_PIN(DSP_PWM9A_GPIO, DSP_PWM9A_GPIOMUX);
    BOARD_CONFIG_EPWM_PIN(DSP_PWM9B_GPIO, DSP_PWM9B_GPIOMUX);

    /* PWM10 pins / PWM10 引脚初始化 */
    BOARD_CONFIG_EPWM_PIN(DSP_PWM10A_GPIO, DSP_PWM10A_GPIOMUX);
    BOARD_CONFIG_EPWM_PIN(DSP_PWM10B_GPIO, DSP_PWM10B_GPIOMUX);

    /* PWM11 pins / PWM11 引脚初始化 */
    BOARD_CONFIG_EPWM_PIN(DSP_PWM11A_GPIO, DSP_PWM11A_GPIOMUX);
    BOARD_CONFIG_EPWM_PIN(DSP_PWM11B_GPIO, DSP_PWM11B_GPIOMUX);

    /* PWM12 pins / PWM12 引脚初始化 */
    BOARD_CONFIG_EPWM_PIN(DSP_PWM12A_GPIO, DSP_PWM12A_GPIOMUX);
    BOARD_CONFIG_EPWM_PIN(DSP_PWM12B_GPIO, DSP_PWM12B_GPIOMUX);
}
