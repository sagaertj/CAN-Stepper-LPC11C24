#define SERIALBUFSIZE				41

#define SWVERSION 	"V1.0"
#define SWDATE 		__DATE__


#define HOME_0_MASK			(1<<2)	 // P3_2
#define HOME_1_MASK			(1<<10)	 // P1_10


#define DEFAULT_SPEED		5000
#define DEFAULT_ACC			5000
#define DEFAULT_DEC			5000

#define DEFAULT_CANID		0x20

//TxTCR bits
#define TCR_STOP  0
#define TCR_RUN   1
#define TCR_RESET 2

//TxMCR bits
#define MCR_MR0I 0x01
#define MCR_MR0R 0x02
#define MCR_MR0S 0x04
#define MCR_MR1I 0x08
#define MCR_MR1R 0x10
#define MCR_MR1S 0x20
#define MCR_MR2I 0x40
#define MCR_MR2R 0x80
#define MCR_MR2S 0x100
#define MCR_MR3I 0x200
#define MCR_MR3R 0x400
#define MCR_MR3S 0x800

#define EMR_EMC0_SET			0x10
#define EMR_EMC0_CLR			0x20
#define EMR_EMC0_TOGGLE 		0x30




