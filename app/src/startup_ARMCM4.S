	.syntax	unified
	.arch	armv7-m

	.section .stack
	.align	3
#ifdef __STACK_SIZE
	.equ	Stack_Size, __STACK_SIZE
#else
	.equ	Stack_Size, 0x00000400
#endif
	.globl	__StackTop
	.globl	__StackLimit
__StackLimit:
	.space	Stack_Size
	.size	__StackLimit, . - __StackLimit
__StackTop:
	.size	__StackTop, . - __StackTop

	.section .heap
	.align	3
#ifdef __HEAP_SIZE
	.equ	Heap_Size, __HEAP_SIZE
#else
	.equ	Heap_Size, 0x00000C00
#endif
	.globl	__HeapBase
	.globl	__HeapLimit
__HeapBase:
	.if	Heap_Size
	.space	Heap_Size
	.endif
	.size	__HeapBase, . - __HeapBase
__HeapLimit:
	.size	__HeapLimit, . - __HeapLimit

	.section .vectors
	.align	2
	.globl	__Vectors
__Vectors:
	.long	__StackTop            /* Top of Stack */
	.long	Reset_Handler         /* Reset Handler */
	.long	NMI_Handler           /* NMI Handler */
	.long	HardFault_Handler     /* Hard Fault Handler */
	.long	MemManage_Handler     /* MPU Fault Handler */
	.long	BusFault_Handler      /* Bus Fault Handler */
	.long	UsageFault_Handler    /* Usage Fault Handler */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	SVC_Handler           /* SVCall Handler */
	.long	DebugMon_Handler      /* Debug Monitor Handler */
	.long	0                     /* Reserved */
	.long	PendSV_Handler        /* PendSV Handler */
	.long	SysTick_Handler       /* SysTick Handler */

	/* External interrupts */
	.long	  	WWDG_IRQHandler					/* 0: Window WatchDog              */                                        
  	.long     PVD_IRQHandler                  	    /* 1: PVD through EXTI Line detection */                        
  	.long     TAMP_STAMP_IRQHandler          /* 2: Tamper and TimeStamps through the EXTI line */            
  	.long     RTC_WKUP_IRQHandler             /* 3: RTC Wakeup through the EXTI line */                      
  	.long     FLASH_IRQHandler                  	/* 4: FLASH                        */                                          
  	.long     RCC_IRQHandler                    	/* 5: RCC                          */                                            
  	.long     EXTI0_IRQHandler                  	/* 6: EXTI Line0                   */                        
  	.long     EXTI1_IRQHandler                 	/* 7: EXTI Line1                   */                          
  	.long     EXTI2_IRQHandler                  	/* 8: EXTI Line2                   */                          
  	.long     EXTI3_IRQHandler                 	/* 9: EXTI Line3                   */                          
  	.long     EXTI4_IRQHandler                  	/* 10: EXTI Line4                   */                          
  	.long     DMA1_Stream0_IRQHandler       /* 11: DMA1 Stream 0                */                  
  	.long     DMA1_Stream1_IRQHandler       /* 12: DMA1 Stream 1                */                   
  	.long     DMA1_Stream2_IRQHandler       /* 13: DMA1 Stream 2                */                   
  	.long     DMA1_Stream3_IRQHandler       /* 14: DMA1 Stream 3                */                   
  	.long     DMA1_Stream4_IRQHandler       /* 15: DMA1 Stream 4                */                   
  	.long     DMA1_Stream5_IRQHandler       /* 16: DMA1 Stream 5                */                   
  	.long     DMA1_Stream6_IRQHandler      	/* 17:DMA1 Stream 6                */                   
  	.long     ADC_IRQHandler                    	/* 18:ADC1, ADC2 and ADC3s         */                   
  	.long     0               				  				/* Reserved                      */                         
  	.long     0              					 			/* Reserved                     */                          
  	.long     0                                 				/* Reserved                     */                          
  	.long     0                                 				/* Reserved                     */                          
  	.long     EXTI9_5_IRQHandler               	/* 23: External Line[9:5]s          */                          
  	.long     TIM1_BRK_TIM9_IRQHandler    	/* 24: TIM1 Break and TIM9          */         
  	.long     TIM1_UP_TIM10_IRQHandler      /* 25: TIM1 Update and TIM10        */         
  	.long     TIM1_TRG_COM_TIM11_IRQHandler     /* 26: TIM1 Trigger and Commutation and TIM11 */
  	.long     TIM1_CC_IRQHandler                /* 27: TIM1 Capture Compare         */                          
  	.long     TIM2_IRQHandler                  	/* 28: TIM2                         */                   
  	.long     TIM3_IRQHandler                   /* 29: TIM3                         */                   
  	.long     TIM4_IRQHandler                   /* 30: TIM4                         */                   
  	.long     I2C1_EV_IRQHandler                /* 31: I2C1 Event                   */                          
  	.long     I2C1_ER_IRQHandler                /* 32: I2C1 Error                   */                          
  	.long     I2C2_EV_IRQHandler                /* 33: I2C2 Event                   */                          
  	.long     I2C2_ER_IRQHandler                /* 34: I2C2 Error                   */                            
  	.long     SPI1_IRQHandler                   /* 35: SPI1                         */                   
  	.long     SPI2_IRQHandler                   /* 36: SPI2                         */                   
  	.long     USART1_IRQHandler                 /* 37: USART1                       */                   
  	.long     USART2_IRQHandler                 /* 38: USART2                       */                   
  	.long     0               			/* Reserved                       */                   
  	.long     EXTI15_10_IRQHandler              /* 40: External Line[15:10]s        */                          
  	.long     RTC_Alarm_IRQHandler              /* 41: RTC Alarm (A and B) through EXTI Line */                 
  	.long     OTG_FS_WKUP_IRQHandler            /* 42: USB OTG FS Wakeup through EXTI line */                       
  	.long     0                                 /* Reserved     				  */         
 	.long     0                                 /* Reserved       			  */         
  	.long     0                                 /* Reserved 					  */
  	.long     0                                 /* Reserved                     */                          
  	.long     DMA1_Stream7_IRQHandler           /* 47: DMA1 Stream7                 */                          
  	.long     0                                 /* Reserved                     */                   
  	.long     SDIO_IRQHandler                   /* 49: SDIO                         */                   
  	.long     TIM5_IRQHandler                   /* 50: TIM5                         */                   
  	.long     SPI3_IRQHandler                   /* 51: SPI3                         */                   
  	.long     0                                 /* Reserved                     */                   
  	.long     0                                 /* Reserved                     */                   
  	.long     0                                 /* Reserved                     */                   
  	.long     0                                 /* Reserved                     */
 	.long     DMA2_Stream0_IRQHandler           /* 56: DMA2 Stream 0                */                   
  	.long     DMA2_Stream1_IRQHandler           /* 57: DMA2 Stream 1                */                   
  	.long     DMA2_Stream2_IRQHandler           /* 58: DMA2 Stream 2                */                   
  	.long     DMA2_Stream3_IRQHandler           /* 59: DMA2 Stream 3                */                   
  	.long     DMA2_Stream4_IRQHandler           /* 60: DMA2 Stream 4                */                   
  	.long     0                    			  /* Reserved                     */                   
  	.long     0              					  /* Reserved                     */                     
  	.long     0              					  /* Reserved                     */                          
  	.long     0             					  /* Reserved                     */                          
  	.long     0              					  /* Reserved                     */                          
  	.long     0              					  /* Reserved                     */                          
  	.long     OTG_FS_IRQHandler                 /* 67: USB OTG FS                   */                   
  	.long     DMA2_Stream5_IRQHandler           /* 68: DMA2 Stream 5                */                   
 	.long     DMA2_Stream6_IRQHandler           /* 69: DMA2 Stream 6                */                   
  	.long     DMA2_Stream7_IRQHandler           /* 70: DMA2 Stream 7                */                   
  	.long     USART6_IRQHandler                 /* 71: USART6                       */                    
  	.long     I2C3_EV_IRQHandler                /* 72: I2C3 event                   */                          
  	.long     I2C3_ER_IRQHandler                /* 73: I2C3 error                   */                          
  	.long     0                                 /* Reserved                     */                   
  	.long     0                                 /* Reserved                     */                   
  	.long     0                                 /* Reserved                     */                         
  	.long     0                                 /* Reserved                     */                   
 	.long     0                                 /* Reserved                     */                   
  	.long     0                                 /* Reserved                     */                   
  	.long     0                                 /* Reserved                     */
  	.long     FPU_IRQHandler                    /* 81: FPU                          */
  	.long     0                                 /* Reserved                     */                   
  	.long     0                                 /* Reserved                     */
  	.long     SPI4_IRQHandler                   /* 84: SPI4                         */
  	.long     SPI5_IRQHandler                   /* 85: SPI5                         */  

	.size	__Vectors, . - __Vectors

	.text
	.thumb
	.thumb_func
	.align	2
	.globl	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
	bl main

Default_Handler:
	b	.
	.size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm

	def_irq_handler	NMI_Handler
	def_irq_handler	HardFault_Handler
	def_irq_handler	MemManage_Handler
	def_irq_handler	BusFault_Handler
	def_irq_handler	UsageFault_Handler
	def_irq_handler	SVC_Handler
	def_irq_handler	DebugMon_Handler
	def_irq_handler	PendSV_Handler
	def_irq_handler	SysTick_Handler

	def_irq_handler		WWDG_IRQHandler                   /* Window WatchDog              */                                        
  	def_irq_handler     PVD_IRQHandler                    /* PVD through EXTI Line detection */                        
  	def_irq_handler     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */            
  	def_irq_handler     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */                      
  	def_irq_handler     FLASH_IRQHandler                  /* FLASH                        */                                          
  	def_irq_handler     RCC_IRQHandler                    /* RCC                          */                                            
  	def_irq_handler     EXTI0_IRQHandler                  /* EXTI Line0                   */                        
  	def_irq_handler     EXTI1_IRQHandler                  /* EXTI Line1                   */                          
  	def_irq_handler     EXTI2_IRQHandler                  /* EXTI Line2                   */                          
  	def_irq_handler     EXTI3_IRQHandler                  /* EXTI Line3                   */                          
  	def_irq_handler     EXTI4_IRQHandler                  /* EXTI Line4                   */                          
  	def_irq_handler     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */                  
  	def_irq_handler     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */                   
  	def_irq_handler     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */                   
  	def_irq_handler     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */                   
  	def_irq_handler     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */                   
  	def_irq_handler     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */                   
  	def_irq_handler     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */                   
  	def_irq_handler     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */                                           
  	def_irq_handler     EXTI9_5_IRQHandler                /* External Line[9:5]s          */                          
  	def_irq_handler     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */         
  	def_irq_handler     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */         
  	def_irq_handler     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  	def_irq_handler     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */                          
  	def_irq_handler     TIM2_IRQHandler                   /* TIM2                         */                   
  	def_irq_handler     TIM3_IRQHandler                   /* TIM3                         */                   
  	def_irq_handler     TIM4_IRQHandler                   /* TIM4                         */                   
  	def_irq_handler     I2C1_EV_IRQHandler                /* I2C1 Event                   */                          
  	def_irq_handler     I2C1_ER_IRQHandler                /* I2C1 Error                   */                          
  	def_irq_handler     I2C2_EV_IRQHandler                /* I2C2 Event                   */                          
  	def_irq_handler     I2C2_ER_IRQHandler                /* I2C2 Error                   */                            
  	def_irq_handler     SPI1_IRQHandler                   /* SPI1                         */                   
  	def_irq_handler     SPI2_IRQHandler                   /* SPI2                         */                   
  	def_irq_handler    USART1_IRQHandler                 /* USART1                       */                   

  	def_irq_handler     USART2_IRQHandler                 /* USART2                       */                                 
  	def_irq_handler     EXTI15_10_IRQHandler              /* External Line[15:10]s        */                          
  	def_irq_handler     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */                 
  	def_irq_handler     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTIline*/		                                           
  	def_irq_handler     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */                          
  	                   
  	def_irq_handler     SDIO_IRQHandler                   /* SDIO                         */                   
  	def_irq_handler     TIM5_IRQHandler                   /* TIM5                         */                   
  	def_irq_handler     SPI3_IRQHandler                   /* SPI3                         */                   
 	def_irq_handler     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */                   
  	def_irq_handler     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */                   
  	def_irq_handler     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */                   
  	def_irq_handler     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */                   
  	def_irq_handler     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */                                          
  	def_irq_handler     OTG_FS_IRQHandler                 /* USB OTG FS                   */                   
  	def_irq_handler     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */                   
 	def_irq_handler     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */                   
  	def_irq_handler     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */                   
  	def_irq_handler     USART6_IRQHandler                 /* USART6                       */                    
  	def_irq_handler     I2C3_EV_IRQHandler                /* I2C3 event                   */                          
  	def_irq_handler     I2C3_ER_IRQHandler                /* I2C3 error                   */                          
  	def_irq_handler     FPU_IRQHandler                    /* FPU                          */
  	def_irq_handler     SPI4_IRQHandler                   /* SPI4                         */
  	def_irq_handler     SPI5_IRQHandler                   /* SPI5                         */  


	.end
