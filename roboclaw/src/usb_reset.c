#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

int main(int argc, char **argv) {
  const char *filename;
  int fd;
  filename = argv[1];
  fd = open(filename, O_WRONLY);
  if (fd == -1) {
    fprintf(stderr, "Bad filename: %s\n", filename);
  }
  if (ioctl(fd, USBDEVFS_RESET, 0) == -1) {
    fprintf(stderr, "IOCTL error: %d %s\n", errno, strerror(errno));
  }
  close(fd);
  return 0;
}
