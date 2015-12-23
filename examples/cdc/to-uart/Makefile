FX2LIBDIR=../../../
BASENAME = cdc-acm-to-uart
SOURCES=../common/main.c ../common/cdc.c
DEPS=../common/cdc.h ../common/cdc-config.h main-custom.c
INCLUDES=../common -I ./
A51_SOURCES=../common/dscr.a51
PID=0x1004

DSCR_AREA=-Wl"-b DSCR_AREA=0x1e00"

include $(FX2LIBDIR)lib/fx2.mk
