TARGET = multicam
SRCDIR = src
BUILDDIR = build
V4L2LDIR = v4l2loopback
CC = gcc

all: multicam

multicam: $(BUILDDIR) $(V4L2LDIR)
	$(CC) $(SRCDIR)/multicam.c -o $(BUILDDIR)/multicam

$(V4L2LDIR):
	git clone git://github.com/umlaeute/v4l2loopback.git $(V4L2LDIR)
	cd $(V4L2LDIR) && $(MAKE)
	cd $(V4L2LDIR) && sudo $(MAKE) install 
	sudo modprobe v4l2loopback

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

clean: 
	rm -rf $(BUILDDIR)

install:
	sudo cp daemon/multicam /etc/init.d/multicam
	sudo update-rc.d multicam defaults
