#include <sys/uio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <setjmp.h>
#include <pwd.h>
#include <grp.h>
#include <limits.h>
#include <shadow.h>
#include <sys/fsuid.h>
#include<netdb.h>
#include<time.h>
#include<sys/time.h>
#include <locale.h>
#include <sys/utsname.h>
#include <dirent.h>
#include<limits.h>
#include<ctype.h>
#include <strings.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include<sys/un.h>
void err(const char *msg)
  {
          perror("msg:");
          exit(1);
  }
void strerr(const char *msg)
{
    fprintf(stderr,"%s %s\n",msg,strerror(errno));
    exit(1);
}