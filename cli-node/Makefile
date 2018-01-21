DEFINES+=PROJECT_CONF_H=\"project-conf.h\"


ifdef USING_TARGET_CC2530
HAVE_BANKING=1
UIP_CONF_IPV6=1
UIP_CONF_RPL=1
endif

CONTIKI_PROJECT = node

all: $(CONTIKI_PROJECT) 

CONTIKI = ../../..

include $(CONTIKI)/Makefile.include
