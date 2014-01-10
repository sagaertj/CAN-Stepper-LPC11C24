#ifndef _CANCMDS_H_
#define _CANCMDS_H_

enum CANCMD			//First Databyte byte of CAN Frame
{
	MSTOP=0,			// STOP
	JOGFWD,			// Step forward
	JOGBWD,			// Step backward
	MOVEABS,			// Move to Absolute position ( using ACCDECE and VRUN )
	MOVEREL,
	SYNC,
	GOHOME,
	FINDHOME,
	VRUN,
	ACCDEC,
	MAXACCDEC,
	VMIN,
	VMAX,
	HOMESWCFG,
	QUIRYCFG,
	QUIRYPOS,
	QUIRYSTAT
};


#endif
