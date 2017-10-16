//Grid Eye AMG8853

#define AMG8853_address     0b1101000
#define GRID_SIDE           8
#define NUM_CELLS           GRID_SIDE*GRID_SIDE

#define REG_PCLT	0x00
#define REG_RST		0x01
#define REG_FPSC	0x02
#define REG_INTC	0x03
#define REG_STAT	0x04
#define REG_SCLR	0x05
#define REG_AVE		0x07
#define REG_INTHL	0x08
#define REG_INTHH	0x09
#define REG_INTLL	0x0A
#define REG_INTLH	0x0B
#define REG_INHYSL	0x0C
#define REG_INHYSH	0x0D
#define REG_TOOL	0x0E
#define REG_TOOH	0x0F
#define REG_INT0	0x10
#define REG_INT1	0x11
#define REG_INT2	0x12
#define REG_INT3	0x13
#define REG_INT4	0x14
#define REG_INT5	0x15
#define REG_INT6	0x16
#define REG_INT7	0x17
#define REG_PIXL	0x80

const int MSG_ENV_INDEX = 0;
const int MSG_GRID_INDEX = 1;

int grid_lv[NUM_CELLS] = {0};
