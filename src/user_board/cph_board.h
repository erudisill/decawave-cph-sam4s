#ifndef _CPH_BOARD_H
#define _CPH_BOARD_H

#include "compiler.h"
#include "system_sam4s.h"
#include "exceptions.h"
#include "pio.h"

//#define REV_A_02
#define REV_A_03

#ifdef REV_A_02
#define BOARD_REV_MAJOR		0x0A
#define BOARD_REV_MINOR		0x02
#endif
#ifdef REV_A_03
#define BOARD_REV_MAJOR		0x0A
#define BOARD_REV_MINOR		0x03
#endif


/** Board oscillator settings */
#define BOARD_FREQ_SLCK_XTAL        (32768U)
#define BOARD_FREQ_SLCK_BYPASS      (32768U)
#define BOARD_FREQ_MAINCK_XTAL      (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS    (12000000U)

/** Master clock frequency */
#define BOARD_MCK                   CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
#define BOARD_OSC_STARTUP_US   15625

/** Name of the board */
#define BOARD_NAME "CPH-BOARD"
/** Board definition */
#define cphBoard
/** Family definition (already defined) */
#define sam4s
/** Core definition */
#define cortexm4

/*----------------------------------------------------------------------------*/
/*	CONSOLE																	  */
/*----------------------------------------------------------------------------*/

//#define CONSOLE_UART        UART0
//#define CONSOLE_UART_ID     ID_UART0
//
//#define PINS_CONSOLE_PIO 		PIOA
//#define PINS_CONSOLE_TYPE 	PIO_PERIPH_A
//#define PINS_CONSOLE_MASK 	PIO_PA9A_URXD0|PIO_PA10A_UTXD0
//#define PINS_CONSOLE_ATTR 	PIO_DEFAULT

#define CONSOLE_UART        UART1
#define CONSOLE_UART_ID     ID_UART1

#define PINS_CONSOLE_PIO 	PIOB
#define PINS_CONSOLE_TYPE 	PIO_PERIPH_A
#define PINS_CONSOLE_MASK 	PIO_PB2A_URXD1|PIO_PB3A_UTXD1
#define PINS_CONSOLE_ATTR 	PIO_DEFAULT

/*----------------------------------------------------------------------------*/
/*	LEDS																	  */
/*----------------------------------------------------------------------------*/

#define LED_STATUS0_IDX		PIO_PC12_IDX

#define PINS_LED0_PIO		PIOC
#define PINS_LED0_TYPE		PIO_OUTPUT_0
#define PINS_LED0_MASK		PIO_PC12
#define PINS_LED0_ATTR		PIO_DEFAULT


#define LED_STATUS1_IDX		PIO_PC13_IDX

#define PINS_LED1_PIO		PIOC
#define PINS_LED1_TYPE		PIO_OUTPUT_0
#define PINS_LED1_MASK		PIO_PC13
#define PINS_LED1_ATTR		PIO_DEFAULT

#define LED_GROUP_PIO		PIOC
#define LED_GROUP_MASK		(PINS_LED0_MASK | PINS_LED1_MASK)



/*----------------------------------------------------------------------------*/
/*	DECAWAVE																  */
/*----------------------------------------------------------------------------*/


// Decawave SPI
#define DW_SPI_BAUD_FAST			5000000UL
#define DW_SPI_BAUD_SLOW			1000000UL
#define DW_SPI						SPI
#define DW_SPI_MAX_BLOCK_TIME 		(50 / portTICK_RATE_MS)

#if defined(REV_A_02)
#define DW_CSn_PIO_IDX				PIO_PC4_IDX
#define DW_CSn_PIO_PERIPH			PIO_PERIPH_B
#define DW_CHIP_SELECT				1
#define DW_CHIP_SELECT_VALUE		0x0001
#define DW_NONE_CHIP_SELECT_VALUE   0x0f
#elif defined(REV_A_03)
#define DW_CSn_PIO_IDX				PIO_PA30_IDX
#define DW_CSn_PIO_PERIPH			PIO_PERIPH_B
#define DW_CHIP_SELECT				2
#define DW_CHIP_SELECT_VALUE		0x03
#define DW_NONE_CHIP_SELECT_VALUE   0x0f
#else
error Undefined refision.
#endif


#define DW_DELAY_BEFORE				0x05
#define DW_DELAY_BETWEEN			0x05
#define DW_DELAY_BETWEEN_CS			0x05
#define DW_CLOCK_POLARITY			1
#define DW_CLOCK_PHASE				1


// Decawave WAKEUP
#define DW_WAKEUP_PIO				PIOC
#define DW_WAKEUP_PIO_IDX			PIO_PC7_IDX
#define DW_WAKEUP_MASK				PIO_PC7
#define DW_WAKEUP_TYPE				PIO_OUTPUT_0
#define DW_WAKEUP_ATTR				(PIO_DEFAULT)

// Decawave RSTn - input, no pull up/down
#define DW_RSTn_PIO					PIOC
#define DW_RSTn_PIO_ID				ID_PIOC
#define DW_RSTn_PIO_IDX				PIO_PC10_IDX
#define DW_RSTn_MASK				PIO_PC10
#define DW_RSTn_TYPE				PIO_INPUT
#define DW_RSTn_ATTR				(PIO_IT_RISE_EDGE | PIO_DEFAULT)
#define DW_RSTn_FLAGS				(DW_RSTn_TYPE | DW_RSTn_ATTR)

// Decawave RSTSWn - tied to a P-CHAN MOSFET
#define DW_RSTSWn_PIO				PIOC
#define DW_RSTSWn_PIO_IDX			PIO_PC11_IDX
#define DW_RSTSWn_MASK				PIO_PC11
#define DW_RSTSWn_TYPE				PIO_OUTPUT_1
#define DW_RSTSWn_ATTR				(PIO_DEFAULT)

// Decawave IRQ
#if defined(REV_A_02)
#define DW_IRQ_PIO					PIOC
#define DW_IRQ_PIO_ID				ID_PIOC
#define DW_IRQ_IDX					PIO_PC0_IDX
#define DW_IRQ_MASK					PIO_PC0
#define DW_IRQ_IRQ					PIOC_IRQn
#elif defined(REV_A_03)
#define DW_IRQ_PIO					PIOA
#define DW_IRQ_PIO_ID				ID_PIOA
#define DW_IRQ_IDX					PIO_PA20_IDX
#define DW_IRQ_MASK					PIO_PA20
#define DW_IRQ_IRQ					PIOA_IRQn
#else
#error Unknown revision.
#endif

#define DW_IRQ_TYPE					PIO_INPUT
#define DW_IRQ_ATTR					(PIO_IT_RISE_EDGE | PIO_DEFAULT)
//#define DW_IRQ_ATTR					(PIO_IT_HIGH_LEVEL | PIO_DEFAULT)
#define DW_IRQ_FLAGS				(DW_IRQ_TYPE | DW_IRQ_ATTR)

#endif  // _CPH_BOARD_H
