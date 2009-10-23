#ifndef _DIAG_H_
#define _DIAG_H_

/* please refer: Documentation/ioctl-number.txt and Documentation/ioctl/
 *  * and choice magic-number */
#define USB_DIAG_IOC_MAGIC 0xFF

#define USB_DIAG_FUNC_IOC_ENABLE_SET	_IOW(USB_DIAG_IOC_MAGIC, 1, int)
#define USB_DIAG_FUNC_IOC_ENABLE_GET	_IOR(USB_DIAG_IOC_MAGIC, 2, int)

#define USB_DIAG_FUNC_IOC_REGISTER_SET  _IOW(USB_DIAG_IOC_MAGIC, 3, char *)
#define USB_DIAG_FUNC_IOC_AMR_SET	_IOW(USB_DIAG_IOC_MAGIC, 4, int)

extern int g_bUsbDiagMode;

#endif /* _DIAG_H_ */
