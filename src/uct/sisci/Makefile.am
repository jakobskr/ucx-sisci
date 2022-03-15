DIS_HOME = /opt/DIS
INCLUDE  = -I$(DIS_HOME)/include -I$(DIS_HOME)/include/dis -I $(DIS_HOME)/src/include

module_LTLIBRARIES    = libuct_sisci.la


libuct_sisci_la_CPPFLAGS = $(BASE_CPPFLAGS) 
libuct_sisci_la_CFLAGS   = $(BASE_CFLAGS) $(INCLUDE) -D_REENTRANT
libuct_sisci_la_LIBADD   = $(top_builddir)/src/ucs/libucs.la \
                        $(top_builddir)/src/uct/libuct.la
libuct_sisci_la_LDFLAGS  = -version-info $(SOVERSION)



noinst_HEADERS = \
	sisci.h \
	sisci_ep.h 

libuct_sisci_la_SOURCES =  \
	sisci_ep.c \
	sisci.c
