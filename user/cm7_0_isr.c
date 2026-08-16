#include "zf_common_headfile.h"
#include "application/camera_helper/app_camera_helper.h"

static void app_uart_service(uart_index_enum uart_index)
{
    uint8 byte;

    if(uart_isr_mask(uart_index) == 0U)
    {
        return;
    }

    while(uart_query_byte(uart_index, &byte) != 0U)
    {
        if(uart_index == UART_6)
        {
            app_camera_helper_on_byte(byte);
        }
    }
}

void pit0_ch0_isr(void)  { pit_isr_flag_clear(PIT_CH0); }
void pit0_ch1_isr(void)  { pit_isr_flag_clear(PIT_CH1); }
void pit0_ch2_isr(void)  { pit_isr_flag_clear(PIT_CH2); }
void pit0_ch10_isr(void) { pit_isr_flag_clear(PIT_CH10); }
void pit0_ch11_isr(void) { pit_isr_flag_clear(PIT_CH11); }
void pit0_ch12_isr(void) { pit_isr_flag_clear(PIT_CH12); }
void pit0_ch13_isr(void) { pit_isr_flag_clear(PIT_CH13); }
void pit0_ch14_isr(void) { pit_isr_flag_clear(PIT_CH14); }
void pit0_ch15_isr(void) { pit_isr_flag_clear(PIT_CH15); }
void pit0_ch16_isr(void) { pit_isr_flag_clear(PIT_CH16); }
void pit0_ch17_isr(void) { pit_isr_flag_clear(PIT_CH17); }
void pit0_ch18_isr(void) { pit_isr_flag_clear(PIT_CH18); }
void pit0_ch19_isr(void) { pit_isr_flag_clear(PIT_CH19); }
void pit0_ch20_isr(void) { pit_isr_flag_clear(PIT_CH20); }
void pit0_ch21_isr(void)
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}

void uart0_isr(void)
{
    if(uart_isr_mask(UART_0) != 0U)
    {
#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
    }
}

void uart1_isr(void) { (void)uart_isr_mask(UART_1); }
void uart2_isr(void) { (void)uart_isr_mask(UART_2); }
void uart3_isr(void) { (void)uart_isr_mask(UART_3); }
void uart4_isr(void) { (void)uart_isr_mask(UART_4); }
void uart5_isr(void) { (void)uart_isr_mask(UART_5); }
void uart6_isr(void) { app_uart_service(UART_6); }

void gpio_0_exti_isr(void)  { }
void gpio_1_exti_isr(void)  { }
void gpio_2_exti_isr(void)  { }
void gpio_3_exti_isr(void)  { }
void gpio_4_exti_isr(void)  { }
void gpio_5_exti_isr(void)  { }
void gpio_6_exti_isr(void)  { }
void gpio_7_exti_isr(void)  { }
void gpio_8_exti_isr(void)  { }
void gpio_9_exti_isr(void)  { }
void gpio_10_exti_isr(void) { }
void gpio_11_exti_isr(void) { }
void gpio_12_exti_isr(void) { }
void gpio_13_exti_isr(void) { }
void gpio_14_exti_isr(void) { }
void gpio_15_exti_isr(void) { }
void gpio_16_exti_isr(void) { }
void gpio_17_exti_isr(void) { }
void gpio_18_exti_isr(void) { }
void gpio_19_exti_isr(void) { }
void gpio_20_exti_isr(void) { }
void gpio_21_exti_isr(void) { }
void gpio_22_exti_isr(void) { }
void gpio_23_exti_isr(void) { }
