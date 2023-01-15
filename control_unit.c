/********************************************************************************
* control_unit.c: Contains static variables and function definitions for 
*                 implementation of an 8-bit control unit.
********************************************************************************/
#include "control_unit.h"

/* Static variables: */
static uint32_t ir;    /* Instruction register, stores next instruction to execute. */
static uint8_t pc;     /* Program counter, stores address to next instruction to fetch. */
static uint8_t mar;    /* Memory address register, stores address for current instruction. */
static uint8_t sr;     /* Status register, stores status bits ISNZVC. */

static uint8_t op_code; /* Stores OP-code, for example LDI, OUT, JMP etc. */
static uint8_t op1;     /* Stores first operand, most often a destination. */
static uint8_t op2;     /* Stores second operand, most often a value or read address. */

static uint8_t reg[CPU_REGISTER_ADDRESS_WIDTH]; /* CPU-registers R0 - R31. */
static enum cpu_state state;                    /* Stores current state. */

/* Temporary memories: */
static uint8_t data_memory[2000];        /* Contains I/O-addresses and stores static variables. */

const static uint32_t program_memory[] = /* Program memory, stores code. */
{
   0x160500, /* JMP 0x05 */
   0x000000, /* NOP */
   0x000000, /* NOP */
   0x000000, /* NOP */
   0x000000, /* NOP */
   0x011001, /* LDI R16, 0x01 */
   0x030010, /* OUT DDRB, R16 */
   0x021110, /* MOV R17, R16 */
   0x011020, /* LDI R16, 0x20 */
   0x030110, /* OUT PORTB, R16 */
   0x030211, /* OUT PINB, R17 */
   0x041002, /* IN R16, PINB */
   0x160500  /* JMP 0x05 */
};

/********************************************************************************
* control_unit_reset: Resets control unit registers and corresponding program.
********************************************************************************/
void control_unit_reset(void)
{
   ir = 0x00;
   pc = 0x00;
   mar = 0x00;
   sr = 0x00;

   op_code = 0x00;
   op1 = 0x00;
   op2 = 0x00;
   state = CPU_STATE_FETCH; 

   for (uint8_t i = 0; i < CPU_REGISTER_ADDRESS_WIDTH; ++i)
   {
      reg[i] = 0x00;
   }

   for (uint16_t i = 0; i < 2000; ++i)
   {
      data_memory[i] = 0x00;
   }

   /* Later: Clear the stack at reset! */
   return;
}

/********************************************************************************
* control_unit_run_next_state: Runs next state in the CPU instruction cycle:
********************************************************************************/
void control_unit_run_next_state(void)
{
   switch (state)
   {
      case CPU_STATE_FETCH:
      {
         ir = program_memory[pc];   /* Fetches next instruction from program memory. */
         mar = pc;                  /* Stores address to current instruction. */
         pc++;                      /* Program counter points to next instruction. */
         state = CPU_STATE_DECODE;  /* The instruction will be decoded during next state. */
         break;
      }
      case CPU_STATE_DECODE:
      {        
         op_code = ir >> 16;        /* Assigns content of instruction register, bit 23 down to 16. */
         op1 = ir >> 8;             /* Assigns content of instruction register, bit 15 down to 8. */
         op2 = ir;                  /* Assigns content of instruction register, bit 7 down to 0. */
         state = CPU_STATE_EXECUTE; /* The instruction will be executed during next state. */
         break;
      }
      case CPU_STATE_EXECUTE:
      {
         switch (op_code) /* Checks the OP code.*/
         {
            case NOP:
            {
               break; /* NOP => do nothing. */
            }
            case LDI:
            {
               reg[op1] = op2; /* LDI R16, 0x01 => op_code = LDI, op1 = R16, op2 = 0x01 */
               break;
            }
            case MOV:
            {
               reg[op1] = reg[op2]; /* MOV R17, R16 => op_code = MOV, op1 = R17, op2 = R16 */
               break;
            }
            case OUT:
            {
               data_memory[op1] = reg[op2]; /* OUT DDRB, R16 => op_code = OUT, op1 = DDRB, op2 = R16 */
               break;
            }
            case IN:
            {
               reg[op1] = data_memory[op2]; /* IN R16, PINB => op_code = IN, op1 = R16, op2 = PINB */
               break;
            }
            case JMP:
            {
               pc = op1; /* JMP 0x05 => op_code = JMP, op1 = 0x05 */
               break;
            }
            default:
            {
               control_unit_reset(); /* System reset if error occurs. */
               break;
            }
         }

         state = CPU_STATE_FETCH; /* Fetches next instruction during next clock cycle. */
         break;
      }
      default: /* System reset if error occurs. */
      {
         control_unit_reset();
         break;
      }
   }
   return;
}

/********************************************************************************
* control_unit_run_next_state: Runs next CPU instruction cycle, i.e. fetches
*                              a new instruction from program memory, decodes
*                              and executes it.
********************************************************************************/
void control_unit_run_next_instruction_cycle(void)
{
   do
   {
      control_unit_run_next_state();
   } while (state != CPU_STATE_EXECUTE);
   return;
}

/********************************************************************************
* control_unit_print: Prints information about the processor, for instance
*                     current subroutine, instruction, state, content in
*                     CPU-registers and I/O registers DDRB, PORTB and PINB.
********************************************************************************/
void control_unit_print(void)
{
   printf("--------------------------------------------------------------------------------\n");
   /* printf("Current subroutine:\t\t\t\t%s\n", program_memory_subroutine_name(mar)); */
   printf("Current instruction:\t\t\t\t%s\n", cpu_instruction_name(op_code));
   printf("Current state:\t\t\t\t\t%s\n", cpu_state_name(state));

   printf("Program counter:\t\t\t\t%hu\n", pc);

   printf("Instruction register:\t\t\t\t%s ", get_binary((ir >> 16) & 0xFF, 8));
   printf("%s ", get_binary((ir >> 8) & 0xFF, 8));
   printf("%s\n", get_binary(ir & 0xFF, 8));

   printf("Status register (ISNZVC):\t\t\t%s\n\n", get_binary(sr, 6));

   printf("Content in CPU register R16:\t\t\t%s\n", get_binary(reg[R16], 8));
   printf("Content in CPU register R24:\t\t\t%s\n\n", get_binary(reg[R24], 8));

   printf("Content in data direction register DDRB:\t%s\n", get_binary(data_memory[DDRB], 8));
   printf("Content in data register PORTB:\t\t\t%s\n", get_binary(data_memory[PORTB], 8));
   printf("Content in pin input register PINB:\t\t%s\n", get_binary(data_memory[PINB], 8));

   printf("--------------------------------------------------------------------------------\n\n");
   return;
}


