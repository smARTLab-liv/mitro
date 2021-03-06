#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>

#define WIDTH_SMALL 320
#define HEIGHT_SMALL 240

#define WIDTH 640
#define HEIGHT 480

#define NUM_BUFFERS 2

#define FIFO_FILE "/tmp/multicam-fifo"
#define MAX_BUF_SIZE 255


typedef struct {
  void *start;
  size_t length;
} buffer;


int open_device(char* name) {
  int device;
  device = open(name, O_RDWR);
  if (device < 0) {
    printf("Failed to open video device %s.", name);
    exit(EXIT_FAILURE);
  }
  struct v4l2_capability vid_caps;
  if (-1 == ioctl(device, VIDIOC_QUERYCAP, &vid_caps)) {
    printf("Failed to get video capabilities of device %s.\n", name);
    perror("VIDIOC_QUERYCAP");
    exit(EXIT_FAILURE);
  }
  printf("Successfully opened device %s.\n", name);
  return device;
}

void set_resolution(int device, int w, int h, struct v4l2_format format) {
  format.fmt.pix.width = w;
  format.fmt.pix.height = h;
  format.fmt.pix.bytesperline = w;
  format.fmt.pix.sizeimage = w*h*2;
  if (-1 == ioctl(device, VIDIOC_S_FMT, &format)) {
    perror("VIDIOC_S_FMT");
    exit(EXIT_FAILURE);
  }
}

buffer* create_buffers(int device, unsigned int num,  size_t size) {
  buffer *buffers;
  struct v4l2_requestbuffers reqbuf;
  memset (&reqbuf, 0, sizeof (reqbuf));
  reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqbuf.memory = V4L2_MEMORY_MMAP;
  reqbuf.count = num;

  if (-1 == ioctl (device, VIDIOC_REQBUFS, &reqbuf)) {
    perror("VIDIOC_REQBUFS");
    exit (EXIT_FAILURE);
  }

  if (reqbuf.count < num) {
    printf ("Could not create enough buffers (%d of %d).\n", reqbuf.count, num);
    exit (EXIT_FAILURE);
  }

  buffers = calloc(reqbuf.count, sizeof(buffer));
  assert(NULL != buffers);

  int i;
  for (i = 0; i < reqbuf.count; i++) {
    struct v4l2_buffer buf;
    memset (&buf, 0, sizeof(buf));
    buf.type = reqbuf.type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (-1 == ioctl (device, VIDIOC_QUERYBUF, &buf)) {
      perror ("VIDIOC_QUERYBUF");
      exit (EXIT_FAILURE);
    }

    // printf("buffer length: %d", buf.length);

    buffers[i].length = buf.length; /* remember for munmap() */

    buffers[i].start = mmap (NULL, buf.length,
			     PROT_READ | PROT_WRITE, /* recommended */
			     MAP_SHARED,             /* recommended */
			     device, buf.m.offset);

    if (MAP_FAILED == buffers[i].start) {
      /* If you do not exit here you should unmap() and free()
	 the buffers mapped so far. */
      perror ("mmap");
      exit (EXIT_FAILURE);
    }
  }
  // queue all buffers
  for (i = 0; i < reqbuf.count; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (-1 == ioctl(device, VIDIOC_QBUF, &buf)) {
      perror("VIDIOC_QBUF");
    } else {
      // printf("Q buffer #%d.\n", buf.index);
    }
  }
  return buffers;
}

void kill_buffers(buffer *buffers, int nbuffers){
  int i;
  for (i = 0; i < nbuffers; i++)
     munmap(buffers[i].start, buffers[i].length);
}

void start_streamin(int device) {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl (device, VIDIOC_STREAMON, &type)) {
    perror ("VIDIOC_STREAMON");
    exit(EXIT_FAILURE);
  }
}

void stop_streamin(int device) {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl (device, VIDIOC_STREAMOFF, &type)) {
    perror ("VIDIOC_STREAMOFF");
    exit(EXIT_FAILURE);
  }
}

struct v4l2_buffer deq_buffer(int device) {
  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
    
  if (-1 == ioctl(device, VIDIOC_DQBUF, &buf)) {
    perror("VIDIOC_DQBUF");
  } else {
    //    printf("DEQ buffer #%d.\n", buf.index);
  }
  return buf;
}


void q_buffer(int device, struct v4l2_buffer *buf) {
  if (-1 == ioctl(device, VIDIOC_QBUF, buf)) {
    perror("VIDIOC_QBUF");
  } else {
    //    printf("Q buffer #%d.\n", buf->index);
  }
}

int main(int argc, char**argv)
{

  if ( argc < 4 ) {
    printf("usage: %s video_in_1 video_in_2 video_out\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  char* devin1_name = argv[1];
  char* devin2_name = argv[2];
  char* devout_name = argv[3];


  // open fifo pipe

  char fifobuf[MAX_BUF_SIZE];
  int fifofd;
  if ( -1 ==  mkfifo(FIFO_FILE, S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH)) {
    if (errno != EEXIST) {
      perror("mkfifo");
      exit(1);
    }
  }
  fifofd = open(FIFO_FILE, O_RDONLY | O_NONBLOCK);


  // start video streams

  int devin1, devin2, devout;

  // input 1
  devin1 = open_device(devin1_name);
 
  // TODO this is a hack to get the standart values in the format
  struct v4l2_format vid_format;
  memset(&vid_format, 0, sizeof(vid_format));
  vid_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(devin1, VIDIOC_G_FMT, &vid_format)) {
    perror("VIDIOC_G_FMT");
    exit(EXIT_FAILURE);
  }
  set_resolution(devin1, WIDTH, HEIGHT, vid_format);
  buffer *buffers1;
  buffers1 = create_buffers(devin1, NUM_BUFFERS, WIDTH*HEIGHT*2);
  start_streamin(devin1);

  // input 2
  devin2 = open_device(devin2_name);
  vid_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  set_resolution(devin2, WIDTH_SMALL, HEIGHT_SMALL, vid_format);
  //set_resolution(devin2, WIDTH, HEIGHT, vid_format);
  
  buffer *buffers2;
  buffers2 = create_buffers(devin2, NUM_BUFFERS, WIDTH_SMALL*HEIGHT_SMALL*2);
  start_streamin(devin2);

  // output
  devout = open_device(devout_name);
  vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  set_resolution(devout, WIDTH, HEIGHT, vid_format);
  

  // output buffer
  __u8*buffer; 
  size_t sizeimage = WIDTH*HEIGHT*2; //vid_format.fmt.pix.sizeimage;
  buffer=(__u8*)malloc(sizeimage);
  memset(buffer, 0, sizeimage);

  int view;
  view = 0;

  while (1) {
    // read fifo pipe
    int numread;
    numread = read(fifofd, fifobuf, MAX_BUF_SIZE);
    if ( numread >  0 )
      printf("read %d bytes from fifo.\n", numread);

    if ( numread == 2 ) {
      int t;
      t = fifobuf[0] - 49;
      if (t > -1 && t < 4 && t != view) {
	if ( t < 2 && view >= 2 ) {
	  stop_streamin(devin1);
	  stop_streamin(devin2);

	  kill_buffers(buffers1, NUM_BUFFERS);
	  kill_buffers(buffers2, NUM_BUFFERS);

	  close(devin1);
	  close(devin2);

	  usleep(100);

	  devin1 = open_device(devin1_name);
	  devin2 = open_device(devin2_name);


	  memset(&vid_format, 0, sizeof(vid_format));
	  vid_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	  if (-1 == ioctl(devin1, VIDIOC_G_FMT, &vid_format)) {
	    perror("VIDIOC_G_FMT");
	    exit(EXIT_FAILURE);
	  }

	  set_resolution(devin1, WIDTH, HEIGHT, vid_format);
	  buffers1 = create_buffers(devin1, NUM_BUFFERS, WIDTH*HEIGHT*2);
 
	  set_resolution(devin2, WIDTH_SMALL, HEIGHT_SMALL, vid_format);
	  buffers2 = create_buffers(devin2, NUM_BUFFERS, WIDTH_SMALL*HEIGHT_SMALL*2);
	  
	  start_streamin(devin1);
	  start_streamin(devin2);
	} else if ( t >= 2 && view < 2) {
	  stop_streamin(devin1);
	  stop_streamin(devin2);
	  kill_buffers(buffers1, NUM_BUFFERS);
	  kill_buffers(buffers2, NUM_BUFFERS);

	  close(devin1);
	  close(devin2);

	  usleep(100);

	  devin1 = open_device(devin1_name);
	  devin2 = open_device(devin2_name);

	  memset(&vid_format, 0, sizeof(vid_format));
	  vid_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	  if (-1 == ioctl(devin1, VIDIOC_G_FMT, &vid_format)) {
	    perror("VIDIOC_G_FMT");
	    exit(EXIT_FAILURE);
	  }

	  set_resolution(devin1, WIDTH_SMALL, HEIGHT_SMALL, vid_format);
	  buffers1 = create_buffers(devin1, NUM_BUFFERS, WIDTH_SMALL*HEIGHT_SMALL*2);

	  set_resolution(devin2, WIDTH, HEIGHT, vid_format);
	  buffers2 = create_buffers(devin2, NUM_BUFFERS, WIDTH*HEIGHT*2);
	  
	  start_streamin(devin1);
	  start_streamin(devin2);
	}
	view = t;
/*
	if ( 0 == view ) {
	  largeframe = buffers1[buf1.index].start;
	  smallframe = buffers2[buf2.index].start;
	} else if ( 1 == view ) {
	  largeframe = buffers2[buf2.index].start;
	  smallframe = buffers1[buf1.index].start;
	} else if ( 2 == view ) {
	  largeframe = buffers1[buf1.index].start;
	  smallframe = NULL;
	} else if ( 3 == view ) {
	  largeframe = buffers2[buf2.index].start;
	  smallframe = NULL;
	}
*/

	printf("Switched to view: %d.\n", view);
      }
    }
    



    // deque buffers
    struct v4l2_buffer buf1;
    struct v4l2_buffer buf2;
    buf1 = deq_buffer(devin1);
    buf2 = deq_buffer(devin2);

    void *largeframe;
    void *smallframe;

    if ( 0 == view ) {
      largeframe = buffers1[buf1.index].start;
      smallframe = buffers2[buf2.index].start;
    } else if ( 1 == view ) {
      largeframe = buffers1[buf1.index].start;
      smallframe = NULL;
    } else if ( 2 == view ) {
      largeframe = buffers2[buf2.index].start;
      smallframe = buffers1[buf1.index].start;
    } else if ( 3 == view ) {
      largeframe = buffers2[buf2.index].start;
      smallframe = NULL;
    }

    if ( NULL != largeframe )
      memcpy(buffer, largeframe, WIDTH*HEIGHT*2);

    
    if ( NULL != smallframe ) {
      int* b1;
      int* b2;
      b1 = (int*) smallframe;
      b2 = (int*) buffer;
      int x, y;
      int i0, i1, i2, i3, o0, o1, o2, o3, o4,  r;

      for (y=0; y<HEIGHT_SMALL; y+=2) {
	for (x=0; x<WIDTH_SMALL/2; x+=2) {
	
	  i0 = b1[y*WIDTH_SMALL/2 + x];
	  i1 = b1[y*WIDTH_SMALL/2 + x + 1];
	  i2 = b1[(y+1)*WIDTH_SMALL/2 + x];
	  i3 = b1[(y+1)*WIDTH_SMALL/2 + x + 1];

	  o0 = (0x000000ff & i0) + (0x000000ff & i1) + (0x000000ff & i2) + (0x000000ff & i3); 
	  o1 = (0x000000ff & i0>>8) + (0x000000ff & i1>>8) + (0x000000ff & i2>>8) + (0x000000ff & i3>>8); 
	  o2 = (0x000000ff & i0>>16) + (0x000000ff & i1>>16) + (0x000000ff & i2>>16) + (0x000000ff & i3>>16); 
	  o3 = (0x000000ff & i0>>24) + (0x000000ff & i1>>24) + (0x000000ff & i2>>24) + (0x000000ff & i3>>24); 

	  r = 0x000000ff & o0 >> 2 | 
	    0x0000ff00 & (o1 >> 2) << 8 | 
	    0x00ff0000 & (o2 >> 2) << 16 | 
	    0xff000000 & (o3 >> 2) << 24;

	  b2[y/2*WIDTH/2 + x/2] = r;
	}
      }
    }

    write(devout, buffer, sizeof(__u8) * sizeimage);

    q_buffer(devin1, &buf1);
    q_buffer(devin2, &buf2);


  }

  /* Cleanup. */

  stop_streamin(devin1);
  kill_buffers(buffers1, NUM_BUFFERS);
  close(devin1);

  stop_streamin(devin2);
  kill_buffers(buffers2, NUM_BUFFERS);
  close(devin2);  

  close(devout);

  close(fifofd);

  return 0;
}
