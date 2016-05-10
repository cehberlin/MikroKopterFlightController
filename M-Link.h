#ifndef _MLINK_H
#define _MLINK_H

extern unsigned char NewMlinkData;
extern void MlinkParser(unsigned char);
extern void MlinkUartInit(void);
extern void ProcessMlinkData(void);

#endif
