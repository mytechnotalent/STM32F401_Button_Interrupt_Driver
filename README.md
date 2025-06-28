<img src="https://github.com/mytechnotalent/STM32F4_Button_Interrupt_Driver/blob/main/STM32F401_Button_Interrupt_Driver.png?raw=true">

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# STM32F4 Button Interrupt Driver


<br>

# Code
```assembler
/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * An STM32F401 button interrupt driver written entirely in Assembler.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: March 9, 2024
 * UPDATE DATE: June 27, 2025
 */

.syntax unified                                       // use unified assembly syntax
.cpu cortex-m4                                        // target Cortex-M4 core
.fpu softvfp                                          // use software floating point
.thumb                                                // use Thumb instruction set

/**
 * The start address for the .data section defined in linker script.
 */
.word _sdata                                          // start of .data

/**
 * The end address for the .data section defined in linker script.
 */
.word _edata                                          // end of .data

/**
 * The start address for the initialization values of the .data section defined in
 * linker script.
 */
.word _sidata                                         // start of .data init values

/**
 * The start address for the .bss section defined in linker script.
 */
.word _sbss                                           // start of .bss

/**
 * The end address for the .bss section defined in linker script.
 */
.word _ebss                                           // end of .bss

/**
 * Provide weak aliases for each Exception handler to the Default_Handler. As they
 * are weak aliases, any function with the same name will override this definition.
 */
.macro weak name
  .global \name                                       // make symbol global
  .weak \name                                         // mark as weak
  .thumb_set \name, Default_Handler                   // set to Default_Handler
  .word \name                                         // vector entry
.endm

/**
 * Initialize the .isr_vector section. The .isr_vector section contains vector 
 * table.
 */
.section .isr_vector, "a"                             // vector table section

/**
 * The STM32F401RE vector table. Note that the proper constructs must be placed 
 * on this to ensure that it ends up at physical address 0x00000000.
 */
.global isr_vector                                    // export vector table
.type isr_vector, %object                             // object type
isr_vector:
  .word _estack                                       // Initial Stack Pointer
  .word Reset_Handler                                 // Reset Handler
   weak NMI_Handler                                   // NMI Handler
   weak HardFault_Handler                             // HardFault Handler
   weak MemManage_Handler                             // MemManage Handler
   weak BusFault_Handler                              // BusFault Handler
   weak UsageFault_Handler                            // UsageFault Handler
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak SVC_Handler                                   // SVC Handler
   weak DebugMon_Handler                              // Debug Monitor Handler
  .word 0                                             // Reserved
   weak PendSV_Handler                                // PendSV Handler
   weak SysTick_Handler                               // SysTick Handler
  .word 0                                             // Reserved
   weak EXTI16_PVD_IRQHandler                         // EXTI Line 16 Interrupt PVD
   weak TAMP_STAMP_IRQHandler                         // Tamper/TimeStamp Interrupt
   weak EXTI22_RTC_WKUP_IRQHandler                    // RTC Wakeup Interrupt
   weak FLASH_IRQHandler                              // FLASH Global Interrupt
   weak RCC_IRQHandler                                // RCC Global Interrupt
   weak EXTI0_IRQHandler                              // EXTI Line0 Interrupt
   weak EXTI1_IRQHandler                              // EXTI Line1 Interrupt
   weak EXTI2_IRQHandler                              // EXTI Line2 Interrupt
   weak EXTI3_IRQHandler                              // EXTI Line3 Interrupt
   weak EXTI4_IRQHandler                              // EXTI Line4 Interrupt
   weak DMA1_Stream0_IRQHandler                       // DMA1 Stream0 Global Interrupt
   weak DMA1_Stream1_IRQHandler                       // DMA1 Stream1 Global Interrupt
   weak DMA1_Stream2_IRQHandler                       // DMA1 Stream2 Global Interrupt
   weak DMA1_Stream3_IRQHandler                       // DMA1 Stream3 Global Interrupt
   weak DMA1_Stream4_IRQHandler                       // DMA1 Stream4 Global Interrupt
   weak DMA1_Stream5_IRQHandler                       // DMA1 Stream5 Global Interrupt
   weak DMA1_Stream6_IRQHandler                       // DMA1 Stream6 Global Interrupt
   weak ADC_IRQHandler                                // ADC1 Global Interrupt
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak EXTI9_5_IRQHandler                            // EXTI Line[9:5] Interrupts
   weak TIM1_BRK_TIM9_IRQHandle                       // TIM1 Break/TIM9 Global Interrupt
   weak TIM1_UP_TIM10_IRQHandler                      // TIM1 Update/TIM10 Global Interrupt
   weak TIM1_TRG_COM_TIM11_IRQHandler                 // TIM1 T/C/TIM11 Global Interrupt
   weak TIM1_CC_IRQHandler                            // TIM1 Capture Compare Interrupt
   weak TIM2_IRQHandler                               // TIM2 Global Interrupt
   weak TIM3_IRQHandler                               // TIM3 Global Interrupt
   weak TIM4_IRQHandler                               // TIM4 Global Interrupt
   weak I2C1_EV_IRQHandler                            // I2C1 Event Interrupt
   weak I2C1_ER_IRQHandler                            // I2C1 Error Interrupt
   weak I2C2_EV_IRQHandler                            // I2C2 Event Interrupt
   weak I2C2_ER_IRQHandler                            // I2C2 Error Interrupt
   weak SPI1_IRQHandler                               // SPI1 Global Interrupt
   weak SPI2_IRQHandler                               // SPI2 Global Interrupt
   weak USART1_IRQHandler                             // USART1 Global Interrupt
   weak USART2_IRQHandler                             // USART2 Global Interrupt
  .word 0                                             // Reserved
   weak EXTI15_10_IRQHandler                          // EXTI Line[15:10] Interrupts
   weak EXTI17_RTC_Alarm_IRQHandler                   // RTC Alarms EXTI
   weak EXTI18_OTG_FS_WKUP_IRQHandler                 // USB OTG FS Wakeup EXTI
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak DMA1_Stream7_IRQHandler                       // DMA1 Stream7 Global Interrupt
  .word 0                                             // Reserved
   weak SDIO_IRQHandler                               // SDIO Global Interrupt
   weak TIM5_IRQHandler                               // TIM5 Global Interrupt
   weak SPI3_IRQHandler                               // SPI3 Global Interrupt
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak DMA2_Stream0_IRQHandler                       // DMA2 Stream0 Global Interrupt
   weak DMA2_Stream1_IRQHandler                       // DMA2 Stream1 Global Interrupt
   weak DMA2_Stream2_IRQHandler                       // DMA2 Stream2 Global Interrupt
   weak DMA2_Stream3_IRQHandler                       // DMA2 Stream3 Global Interrupt
   weak DMA2_Stream4_IRQHandler                       // DMA2 Stream4 Global Interrupt
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak OTG_FS_IRQHandler                             // USB OTG FS Global Interrupt
   weak DMA2_Stream5_IRQHandler                       // DMA2 Stream5 Global Interrupt
   weak DMA2_Stream6_IRQHandler                       // DMA2 Stream6 Global Interrupt
   weak DMA2_Stream7_IRQHandler                       // DMA2 Stream7 Global Interrupt
   weak USART6_IRQHandler                             // USART6 Global Interrupt
   weak I2C3_EV_IRQHandler                            // I2C3 Event Interrupt
   weak I2C3_ER_IRQHandler                            // I2C3 Error Interrupt
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak SPI4_IRQHandler                               // SPI4 Global Interrupt

/**
 * @brief   This code is called when processor starts execution.
 *
 * @details This is the code that gets called when the processor first
 *          starts execution following a reset event. We first define and init 
 *          the bss section and then define and init the data section, after which
 *          the application supplied main routine is called.
 *
 * @param   None
 * @retval  None
 */
.type Reset_Handler, %function                        // function type
.global Reset_Handler                                 // export symbol
Reset_Handler:
.Reset_Handler_Setup:
  LDR   R4, =_estack                                  // load addr at end of stack R4
  MOV   SP, R4                                        // move addr at end of stack SP
  LDR   R4, =_sdata                                   // copy data seg init flash to SRAM
  LDR   R5, =_edata                                   // copy data seg init flash to SRAM
  LDR   R6, =_sidata                                  // copy data seg init flash to SRAM
  MOVS  R7, #0                                        // zero offset
  B     .Reset_Handler_Loop_Copy_Data_Init            // branch
.Reset_Handler_Copy_Data_Init:
  LDR   R8, [R6, R7]                                  // copy data seg init to regs
  STR   R8, [R4, R7]                                  // copy data seg init tp regs
  ADDS  R7, R7, #4                                    // increment offset
.Reset_Handler_Loop_Copy_Data_Init:
  ADDS  R8, R4, R7                                    // initialize the data segment
  CMP   R8, R5                                        // compare
  BCC   .Reset_Handler_Copy_Data_Init                 // branch if carry is clear
  LDR   R6, =_sbss                                    // copy bss seg init flash to SRAM
  LDR   R8, =_ebss                                    // copy bss seg init flash to SRAM
  MOVS  R7, #0                                        // zero offset
  B     .Reset_Handler_Loop_Fill_Zero_BSS             // branch
.Reset_Handler_Fill_Zero_BSS:
  STR   R7, [R6]                                      // zero fill the bss segment
  ADDS  R6, R6, #4                                    // increment pointer
.Reset_Handler_Loop_Fill_Zero_BSS:
  CMP   R6, R8                                        // compare
  BCC   .Reset_Handler_Fill_Zero_BSS                  // branch if carry is clear
.Reset_Handler_Call_Main:
  BL    main                                          // call main

/**
 * @brief   This code is called when the processor receives an unexpected interrupt.
 *
 * @details This simply enters an infinite loop, preserving the system state for  
 *          examination by a debugger.
 *
 * @param   None
 * @retval  None
 */
.type Default_Handler, %function                      // function type
.global Default_Handler                               // export symbol
Default_Handler:
  BKPT                                                // set processor into debug state
  B.N   Default_Handler                               // infinite loop

/**
 * Initialize the .text section. 
 * The .text section contains executable code.
 */
.section .text                                        // code section

/**
 * @brief  Entry point for initialization and setup of specific functions.
 *
 *         This function is the entry point for initializing and setting up specific 
 *         functions. It calls other functions to enable certain features and then 
 *         enters a loop for further execution.
 *
 * @param  None
 * @retval None
 */
.type main, %function
.global main
main:
.Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.GPIOC_Enable:
  BL    GPIOC_Enable                                  // call function 
.GPIOC_PC13_EXTI_Init:
  BL    GPIOC_PC13_EXTI_Init                          // call function
.Loop:
  BL    Loop                                          // call function
.Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Enables the GPIOC peripheral by setting the corresponding RCC_AHB1ENR bit.
 *
 * @details This function enables the GPIOC peripheral by setting the corresponding RCC_AHB1ENR bit.  
 *          It loads the address of the RCC_AHB1ENR register, retrieves the current value of the 
 *          register, sets the GPIOCEN bit, and stores the updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
GPIOC_Enable:
.GPIOC_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.GPIOC_Enable_Set_RCC_AHB1ENR:
  LDR   R4, =0x40023830                               // load address of RCC_AHB1ENR register
  LDR   R5, [R4]                                      // load value inside RCC_AHB1ENR register
  ORR   R5, #(1<<2)                                   // set the GPIOCEN bit
  STR   R5, [R4]                                      // store value into RCC_AHB1ENR register
.GPIOC_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Initializes GPIOC PC13 for EXTI interrupt.
 *
 * @details This function configures GPIOC PC13 for EXTI interrupt. It sets the pin's mode
 *          to input and enables the internal pull-up resistor. Additionally, it enables the
 *          EXTI interrupt for PC13, configures SYSCFG_EXTICR4, and sets the corresponding
 *          EXTI and NVIC settings to enable interrupt handling for PC13.
 * 
 * @param   None
 * @retval  None
 */
GPIOC_PC13_EXTI_Init:
.GPIOC_PC13_EXTI_Init_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.GPIOC_PC13_EXTI_Init_Disable_Global_Interrupts:
  CPSID I                                             // disable global interrupts
.GPIOC_PC13_EXTI_Init_Set_GPIOC_MODER:
  LDR   R4, =0x40020800                               // load address of GPIOC_MODER register
  LDR   R5, [R1]                                      // load value inside GPIOC_MODER register
  AND   R5, #~(1<<27)                                 // clear the MODER13 bit
  AND   R5, #~(1<<26)                                 // clear the MODER13 bit
  STR   R5, [R4]                                      // store value into GPIOC_MODER register
.GPIOC_PC13_EXTI_Init_Set_GPIOC_PUPDR:
  LDR   R4, =0x4002080C                               // load address of GPIOC_PUPDR register
  LDR   R5, [R4]                                      // load value inside GPIOC_PUPDR register
  AND   R5, #~(1<<27)                                 // clear the PUPDR13 bit
  ORR   R5, #(1<<26)                                  // set the PUPDR13 bit
  STR   R5, [R4]                                      // store value into GPIOC_PUPDR register
.GPIOC_PC13_EXTI_Init_Set_RCC_APB2ENR:
  LDR   R4, =0x40023844                               // load address of RCC_ABP2ENR
  LDR   R5, [R4]                                      // load value inside RCC_ABP2ENR register
  ORR   R5, #(1<<14)                                  // set SYSCFGEN bit
  STR   R5, [R4]                                      // store value into RCC_APB2ENR register
.GPIOC_PC13_EXTI_Init_Set_SYSCFG_EXTICR4:
  LDR   R4, =0x40013814                               // load address of SYSCFG_EXTICR4
  LDR   R5, [R1]                                      // load value inside SYSCFG_EXTICR4 register
  ORR   R5, #(1<<5)                                   // set EXTI13 bit
  STR   R5, [R4]                                      // store value into SYSCFG_EXTICR4 register
.GPIOC_PC13_EXTI_Init_Set_EXTI_IMR:
  LDR   R4, =0x40013C00                               // load address of EXTI_IMR register
  LDR   R5, [R4]                                      // load value inside EXTI_IMR register
  ORR   R5, #(1<<13)                                  // set MR13 bit
  STR   R5, [R4]                                      // store value into EXTI_IMR register
.GPIOC_PC13_EXTI_Init_Set_EXTI_FTSR:
  LDR   R4, =0x40013C0C                               // load address of EXTI_FTSR register
  LDR   R5, [R4]                                      // load value inside EXTI_FTSR register
  ORR   R5, #(1<<13)                                  // set TR13 bit
  STR   R5, [R4]                                      // store value into EXTI_IMR register
.GPIOC_PC13_EXTI_Init_Enable_NVIC_IRQ:
  BL    NVIC_EnableIRQ_EXTI15_10                      // call function
.GPIOC_PC13_EXTI_Init_Enable_Global_Interrupts:
  CPSIE I                                             // enable global interrupts
.GPIOC_PC13_EXTI_Init_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   EXTI callback function for button control.
 *
 * @details This EXTI callback function handles a button press on the main board
 *          of the MCU.
 *
 * @param   None
 * @retval  None
 */
EXTI_Callback:
.EXTI_Callback_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.EXTI_Callback_NOP:
  NOP                                                 // no op to be a placeholder for practical func
.EXTI_Callback_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Enable NVIC (Nested Vectored Interrupt Controller) for EXTI15_10 interrupts.
 *
 * @details This function enables the NVIC for EXTI15_10 interrupts. It specifically targets
 *          the NVIC_ISER1 register, which controls interrupts 32 to 63, and sets the bit
 *          corresponding to EXTI15_10 (bit 8) to enable the interrupt handling for EXTI lines
 *          15 to 10.
 *
 * @param   None
 * @retval  None
 */
NVIC_EnableIRQ_EXTI15_10:
.NVIC_EnableIRQ_EXTI15_10_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.NVIC_EnableIRQ_EXTI15_10_Set_NVIC_ISER1:
  LDR   R4, =0xE000E104                               // NVIC_ISER1, p 683 M7 Arch ref manual, ISER1 interrupts 32-63
  LDR   R5, [R4]                                      // load value inside NVIC_ISER1 register
  ORR   R5, #(1<<8)                                   // EXTI15_10 (p 204 Ref Manual), p 210 M4 PM, ISER1 8 = EXTI15_10 
  STR   R5, [R4]                                      // store value into R0
.NVIC_EnableIRQ_EXTI15_10_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Infinite loop function.
 *
 * @details This function implements an infinite loop using an unconditional branch
 *          (B) statement. It is designed to keep the program running indefinitely 
 *          by branching back to itself.
 *
 * @param   None
 * @retval  None
 */
Loop:
  B     .                                             // branch infinite loop

/**
 * Test data and constants.
 * The .rodata section is used for constants and static data.
 */
.section .rodata                                      // read-only data section

/**
 * Initialized global data.
 * The .data section is used for initialized global or static variables.
 */
.section .data                                        // data section

/**
 * Uninitialized global data.
 * The .bss section is used for uninitialized global or static variables.
 */
.section .bss                                         // BSS section
```

<br>

## License
[MIT](https://raw.githubusercontent.com/mytechnotalent/STM32F4_Button_Interrupt_Driver/main/LICENSE)
