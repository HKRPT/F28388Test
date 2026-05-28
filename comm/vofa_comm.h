#ifndef VOFA_COMM_H
#define VOFA_COMM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 * VOFA JustFloat communication configuration.
 * VOFA JustFloat 通讯配置。
 *
 * One frame format:
 *   float ch0, float ch1, ..., float chN, 00 00 80 7F
 *
 * 一帧数据格式：
 *   float ch0, float ch1, ..., float chN, 00 00 80 7F
 *------------------------------------------------------------------------------*/
#define VOFA_COMM_CHANNEL_COUNT         8U
#define VOFA_COMM_SEND_DECIMATION       100U
#define VOFA_COMM_TAIL_SIZE             4U
#define VOFA_COMM_RX_VALUE_MAX_LEN      24U

/* Initialize VOFA communication cache.
 * 初始化 VOFA 通讯缓存。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 *
 * This function does not initialize SCI or USB. The low-level port is kept
 * outside this file.
 *
 * 本函数不初始化 SCI 或 USB，底层端口初始化放在其它文件中。
 */
void VOFA_CommInit(void);

/* Set one VOFA channel value.
 * 设置一个 VOFA 通道值。
 *
 * Input:
 *   channel - Channel index, range 0 ~ VOFA_COMM_CHANNEL_COUNT - 1.
 *             通道下标，范围 0 ~ VOFA_COMM_CHANNEL_COUNT - 1。
 *   value   - Floating point value to send.
 *             要发送的浮点值。
 * Output:
 *   None.
 */
void VOFA_CommSetChannel(uint16_t channel, float value);

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
void VOFA_CommSetChannels(const float *data, uint16_t count);

/* Mark a frame request from control ISR.
 * 在控制中断中置位一帧发送请求。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 *
 * This function only sets a pending flag. It does not send data and must not
 * block.
 *
 * 本函数只置位 pending 标志，不发送数据，也不能阻塞。
 */
void VOFA_CommMarkFrameFromISR(void);

/* VOFA background communication task.
 * VOFA 后台通讯任务。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 *
 * Call this function in the main while(1) loop. It sends at most one pending
 * frame per call.
 *
 * 在 main 的 while(1) 中调用。本函数每次最多发送一帧待发送数据。
 */
void VOFA_CommTask(void);

/* Feed one received character into VOFA command parser.
 * 向 VOFA 命令解析器喂入一个接收到的字符。
 *
 * Input:
 *   ch - One ASCII character received from SCI/USB.
 *        从 SCI/USB 接收到的一个 ASCII 字符。
 * Output:
 *   None.
 *
 * Supported command frame:
 *   AA<cmd><value>BB
 *
 * First supported commands:
 *   VP: voltage PI Kp
 *   VI: voltage PI Ki
 *   IP: current PI Kp
 *   II: current PI Ki
 *   VT: target voltage
 *   IT: target current
 *
 * 支持的命令帧：
 *   AA<命令><数值>BB
 *
 * 第一批支持的命令：
 *   VP：电压环 Kp
 *   VI：电压环 Ki
 *   IP：电流环 Kp
 *   II：电流环 Ki
 *   VT：目标电压
 *   IT：目标电流
 */
void VOFA_CommRxChar(char ch);

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
void VOFA_CommRxBytes(const uint16_t *buf, uint16_t len);

/* Poll SCIB RX FIFO and feed received bytes into VOFA parser.
 * 轮询 SCIB RX FIFO，并把收到的字节送入 VOFA 解析器。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 */
void VOFA_CommRxTask(void);

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
uint16_t VOFA_CommSendFloatFrame(const float *data, uint16_t count);

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
 *
 * Current implementation is a placeholder. Later it can be changed to call
 * SCIB or USB CDC without changing the VOFA frame API above.
 *
 * 当前实现是占位接口。后续可以在不改变上层 VOFA API 的情况下改为调用 SCIB
 * 或 USB CDC。
 */
uint16_t VOFA_CommWriteBytes(const uint16_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_COMM_H */
