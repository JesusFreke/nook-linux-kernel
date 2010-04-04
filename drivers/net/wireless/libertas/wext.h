/**
  * This file contains definition for IOCTL call.
  */
#ifndef	_LBS_WEXT_H_
#define	_LBS_WEXT_H_

extern struct iw_handler_def lbs_handler_def;
extern struct iw_handler_def mesh_handler_def;

int SendBGScanQuery(struct lbs_private* priv);
int SendDeepSleepCmd(struct lbs_private* priv);
int SendConfirmAwake(struct lbs_private *priv);
int DisassociateAndEnterDS(struct lbs_private* priv);
int EnableBackgroundScan(struct lbs_private* priv, int bgScanEnabled,
    char* apList, int secondsBetweenScans, int scanThresh, 
    int millisecPerChannel);
int AddChannelListTLV(u8* tlvStart, int numChannels, int scanTimeMs);
int AddSsidWildcardTLV(u8* tlvStart, char* ssid);
int SendDeepSleepCmd(struct lbs_private* priv);
int ExitDeepSleepCmd(struct lbs_private* priv);
#endif
