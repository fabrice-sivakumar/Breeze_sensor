# 1 "C:\\Localization\\US localization\\RRDV\\nRF\\Device\\Startup\\ses_startup_nrf_common.s"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "C:\\Localization\\US localization\\RRDV\\nRF\\Device\\Startup\\ses_startup_nrf_common.s"
# 78 "C:\\Localization\\US localization\\RRDV\\nRF\\Device\\Startup\\ses_startup_nrf_common.s"
        .syntax unified
# 101 "C:\\Localization\\US localization\\RRDV\\nRF\\Device\\Startup\\ses_startup_nrf_common.s"
        .global reset_handler
        .global Reset_Handler
        .equ reset_handler, Reset_Handler
        .extern nRFInitialize
        .global afterInitialize

        .section .init.Reset_Handler, "ax"
        .balign 2
        .thumb_func

Reset_Handler:


        bl nRFInitialize





        bl SystemInit
# 162 "C:\\Localization\\US localization\\RRDV\\nRF\\Device\\Startup\\ses_startup_nrf_common.s"
        bl _start





        .weak SystemInit





        .section .init_array, "aw"
        .balign 4
        .word SystemCoreClockUpdate
# 195 "C:\\Localization\\US localization\\RRDV\\nRF\\Device\\Startup\\ses_startup_nrf_common.s"
        .weak HardFault_Handler
        .section .init.HardFault_Handler, "ax"
        .balign 2
        .thumb_func
HardFault_Handler:



        ldr R1, =0xE000ED2C
        ldr R2, [R1]
        cmp R2, #0

hfLoop:
        bmi hfLoop
# 223 "C:\\Localization\\US localization\\RRDV\\nRF\\Device\\Startup\\ses_startup_nrf_common.s"
        tst LR, #4
        ite eq
        mrseq R0, MSP
        mrsne R0, PSP
# 237 "C:\\Localization\\US localization\\RRDV\\nRF\\Device\\Startup\\ses_startup_nrf_common.s"
        orr R2, R2, #0x80000000
        str R2, [R1]




        ldr R1, [R0, #24]
        adds R1, #2
        str R1, [R0, #24]

        bx LR
