/*
 * lock.c
 *
 *  Created on: 28 Mar 2018
 *      Author: Tony
 *      E-Mail: Tony@gmail.com
 */
#include "lock.h"

#ifndef WIN32

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <fcntl.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <signal.h>
#ifdef LFS
/*----------------------------------------------------------
 lfs_lock
   accept:      The name of the device to try to lock
   perform:     Create a lock file if there is not one already using a
                lock file server.
   return:      1 on failure 0 on success
   exceptions:  none
   comments:
----------------------------------------------------------*/
int lfs_lock(const char *filename, int pid) {
  int s;
  int ret;
  int size = 1024;
  char *buffer = malloc(size);
  struct sockaddr_in addr;

  if (!(s = socket(AF_INET, SOCK_STREAM, 0)) > 0) {
    free(buffer);
    return 1;
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(50001);
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");

  if (!connect(s, (struct sockaddr *) &addr, sizeof(addr)) == 0) {
    free(buffer);
    return 1;
  }

  ret = recv(s, buffer, size, 0);
  sprintf(buffer, "lock %s %i\n", filename, pid);
  /* printf( "%s", buffer ); */
  send(s, buffer, strlen(buffer), 0);
  ret = recv(s, buffer, size, 0);

  if (ret > 0) {
    buffer[ret] = '\0';
    /* printf( "Message recieved: %s", buffer ); */
  }

  send(s, "quit\n", strlen("quit\n"), 0);
  close(s);

  /* printf("%s\n", buffer); */
  if (buffer[0] == '2') {
    return 0;
  }

  free(buffer);
  return 1;
}

/*----------------------------------------------------------
 lfs_unlock
   accept:      The name of the device to try to unlock
   perform:     Remove a lock file if there is one using a
                lock file server.
   return:      1 on failure 0 on success
   exceptions:  none
   comments:
----------------------------------------------------------*/
int lfs_unlock(const char *filename, int pid) {
  int s;
  int ret;
  int size = 1024;
  char *buffer = malloc(size);
  struct sockaddr_in addr;

  if (!(s = socket(AF_INET, SOCK_STREAM, 0)) > 0) {
    free(buffer);
    return 1;
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(50001);
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");

  if (!connect(s, (struct sockaddr *) &addr, sizeof(addr)) == 0) {
    free(buffer);
    return 1;
  }

  sprintf(buffer, "unlock %s %i\n", filename, pid);
  /* printf( "%s", buffer ); */
  send(s, buffer, strlen(buffer), 0);
  ret = recv(s, buffer, size, 0);

  if (ret > 0) {
    buffer[ret] = '\0';
    /* printf( "Message recieved: %s", buffer ); */
  }

  send(s, "quit\n", strlen("quit\n"), 0);
  close(s);

  if (buffer[0] == '2') {
    return 0;
  }

  free(buffer);
  return 1;
}
#endif /* LFS */

/*----------------------------------------------------------
 lib_lock_dev_unlock
   accept:      The name of the device to try to unlock
   perform:     Remove a lock file if there is one using a
                lock file server.
   return:      1 on failure 0 on success
   exceptions:  none
   comments:    This is for use with liblockdev which comes with Linux
        distros.  I suspect it will be problematic with embeded
        Linux.   taj
----------------------------------------------------------*/
#ifdef LIBLOCKDEV
int lib_lock_dev_unlock(const char *filename, int pid) {
  if (dev_unlock(filename, pid)) {
    //report("fhs_unlock: Unable to remove LockFile\n");
    return (1);
  }

  return (0);
}
#endif /* LIBLOCKDEV */

/*----------------------------------------------------------
 lib_lock_dev_lock
   accept:      The name of the device to try to lock
                termios struct
   perform:     Create a lock file if there is not one already.
   return:      1 on failure 0 on success
   exceptions:  none
   comments:    This is for use with liblockdev which comes with Linux
        distros.  I suspect it will be problematic with embeded
        Linux.   taj
        One could load the library here rather than link it and
        always try to use this.
----------------------------------------------------------*/
#ifdef LIBLOCKDEV
int lib_lock_dev_lock(const char *filename, int pid) {
  char message[80];
  printf("LOCKING %s\n", filename);

  if (dev_testlock(filename)) {
    //report( "fhs_lock() lockstatus fail\n" );
    return 1;
  }

  if (dev_lock(filename)) {
    sprintf(message,
            "RXTX fhs_lock() Error: creating lock file for: %s: %s\n",
            filename, strerror(errno));
    // report_error( message );
    return 1;
  }

  return (0);
}
#endif /* LIBLOCKDEV */

/*----------------------------------------------------------
 fhs_lock
   accept:      The name of the device to try to lock
                termios struct
   perform:     Create a lock file if there is not one already.
   return:      1 on failure 0 on success
   exceptions:  none
   comments:    This is for linux and freebsd only currently.  I see SVR4 does
                this differently and there are other proposed changes to the
        Filesystem Hierachy Standard
        more reading:
----------------------------------------------------------*/
int fhs_lock(const char *filename, int pid) {
  /*
   * There is a zoo of lockdir possibilities
   * Its possible to check for stale processes with most of them.
   * for now we will just check for the lockfile on most
   * Problem lockfiles will be dealt with.  Some may not even be in use.
   *
   */
  int fd, j;
  char lockinfo[12];
  char file[80], *p;

  j = strlen(filename);
  p = (char *) filename + j;

  /*  FIXME  need to handle subdirectories /dev/cua/...
      SCO Unix use lowercase all the time
          taj
  */
  while (*(p - 1) != '/' && j-- != 1) {
#if defined ( __unixware__ )
    *p = tolower(*p);
#endif /* __unixware__ */
    p--;
  }

  sprintf(file, "%s/LCK..%s", LOCKDIR, p);

  if (check_lock_status(filename)) {
    printf("fhs_lock() lockstatus fail\n");
    return 1;
  }

  fd = open(file, O_CREAT | O_WRONLY | O_EXCL, 0444);

  if (fd < 0) {
    printf(
      "RXTX fhs_lock() Error: creating lock file: %s: %s\n",
      file, strerror(errno));
    return 1;
  }

  sprintf(lockinfo, "%10d\n", (int) getpid());
  printf("fhs_lock: creating lockfile: %s\n", lockinfo);
  write(fd, lockinfo, 11);
  close(fd);
  return 0;
}

/*----------------------------------------------------------
 uucp_lock
   accept:     char * filename.  Device to be locked
   perform:    Try to get a uucp_lock
   return:     int 0 on success
   exceptions: none
   comments:
        The File System Hierarchy Standard
        http://www.pathname.com/fhs/
        UUCP Lock Files
        http://docs.freebsd.org/info/uucp/uucp.info.UUCP_Lock_Files.html
        FSSTND
        ftp://tsx-11.mit.edu/pub/linux/docs/linux-standards/fsstnd/
        Proposed Changes to the File System Hierarchy Standard
        ftp://scicom.alphacdc.com/pub/linux/devlock-0.X.tgz
        "UNIX Network Programming", W. Richard Stevens,
        Prentice-Hall, 1990, pages 96-101.
        There is much to do here.
        1) UUCP style locks (done)
            /var/spool/uucp
        2) SVR4 locks
            /var/spool/locks
        3) FSSTND locks (done)
            /var/lock
        4) handle stale locks  (done except kermit locks)
        5) handle minicom lockfile contents (FSSTND?)
            "     16929 minicom root\n"  (done)
        6) there are other Lock conventions that use Major and Minor
           numbers...
        7) Stevens recommends LCK..<pid>
        most are caught above.  If they turn out to be problematic
        rather than an exercise, we will handle them.
----------------------------------------------------------*/
int uucp_lock(const char *filename, int pid) {
  char lockfilename[80], lockinfo[12];
  char name[80];
  int fd;
  struct stat buf;

  printf("uucp_lock( %s );\n", filename);

  if (check_lock_status(filename)) {
    printf("RXTX uucp check_lock_status true\n");
    return 1;
  }

  if (stat(LOCKDIR, &buf) != 0) {
    printf("RXTX uucp_lock() could not find lock directory.\n");
    return 1;
  }

  if (stat(filename, &buf) != 0) {
    printf("RXTX uucp_lock() could not find device.\n");
    printf("uucp_lock: device was %s\n", name);
    return 1;
  }

  sprintf(lockfilename, "%s/LK.%03d.%03d.%03d",
          LOCKDIR,
          (int) major(buf.st_dev),
          (int) major(buf.st_rdev),
          (int) minor(buf.st_rdev)
         );
  sprintf(lockinfo, "%10d\n", (int) getpid());

  if (stat(lockfilename, &buf) == 0) {
    printf("RXTX uucp_lock() %s is there\n",
           lockfilename);
    return 1;
  }

  fd = open(lockfilename, O_CREAT | O_WRONLY | O_EXCL, 0444);

  if (fd < 0) {
    printf(
      "RXTX uucp_lock() Error: creating lock file: %s\n",
      lockfilename);
    return 1;
  }

  write(fd, lockinfo, 11);
  close(fd);
  return 0;
}

/*----------------------------------------------------------
 check_lock_status
   accept:      the lock name in question
   perform:     Make sure everything is sane
   return:      0 on success
   exceptions:  none
   comments:
----------------------------------------------------------*/
int check_lock_status(const char *filename) {
  struct stat buf;
  /*  First, can we find the directory? */

  if (stat(LOCKDIR, &buf) != 0) {
    printf("check_lock_status: could not find lock directory.\n");
    return 1;
  }

  /*  OK.  Are we able to write to it?  If not lets bail */

  if (check_group_uucp()) {
    printf("check_lock_status: No permission to create lock file.\nplease see: How can I use Lock Files with rxtx? in INSTALL\n");
    return (1);
  }

  /* is the device alread locked */

  if (is_device_locked(filename)) {
    printf("check_lock_status: device is locked by another application\n");
    return 1;
  }

  return 0;

}

/*----------------------------------------------------------
 fhs_unlock
   accept:      The name of the device to unlock
   perform:     delete the lock file
   return:      none
   exceptions:  none
   comments:    This is for linux only currently.  I see SVR4 does this
                differently and there are other proposed changes to the
        Filesystem Hierachy Standard
----------------------------------------------------------*/
void fhs_unlock(const char *filename, int openpid) {
  char file[80], *p;
  int i;

  i = strlen(filename);
  p = (char *) filename + i;

  /*  FIXME  need to handle subdirectories /dev/cua/... */
  while (*(p - 1) != '/' && i-- != 1) {
    p--;
  }

  sprintf(file, "%s/LCK..%s", LOCKDIR, p);

  if (!check_lock_pid(file, openpid)) {
    unlink(file);
    printf("fhs_unlock: Removing LockFile\n");
  } else {
    printf("fhs_unlock: Unable to remove LockFile\n");
  }
}

/*----------------------------------------------------------
 uucp_unlock
   accept:     char *filename the device that is locked
   perform:    remove the uucp lockfile if it exists
   return:     none
   exceptions: none
   comments:   http://docs.freebsd.org/info/uucp/uucp.info.UUCP_Lock_Files.html
----------------------------------------------------------*/
void uucp_unlock(const char *filename, int openpid) {
  struct stat buf;
  char file[80];
  /* FIXME */

  printf("uucp_unlock( %s );\n", filename);

  if (stat(filename, &buf) != 0) {
    /* hmm the file is not there? */
    printf("uucp_unlock() no such device\n");
    return;
  }

  sprintf(file, LOCKDIR"/LK.%03d.%03d.%03d",
          (int) major(buf.st_dev),
          (int) major(buf.st_rdev),
          (int) minor(buf.st_rdev)
         );

  if (stat(file, &buf) != 0) {
    /* hmm the file is not there? */
    printf("uucp_unlock no such lockfile\n");
    return;
  }

  if (!check_lock_pid(file, openpid)) {
    printf("uucp_unlock: unlinking %s\n", file);
    unlink(file);
  } else {
    printf("uucp_unlock: unlinking failed %s\n", file);
  }
}

/*----------------------------------------------------------
 check_lock_pid
   accept:     the name of the lockfile
   perform:    make sure the lock file is ours.
   return:     0 on success
   exceptions: none
   comments:
----------------------------------------------------------*/
int check_lock_pid(const char *file, int openpid) {
  int fd, lockpid;
  char pid_buffer[12];

  fd = open(file, O_RDONLY);

  if (fd < 0) {
    return (1);
  }

  if (read(fd, pid_buffer, 11) < 0) {
    close(fd);
    return (1);
  }

  close(fd);
  pid_buffer[11] = '\0';
  lockpid = atol(pid_buffer);

  /* Native threads JVM's have multiple pids */
  if (lockpid != getpid() && lockpid != getppid() && lockpid != openpid) {
    printf("check_lock_pid: lock = %s pid = %i gpid=%i openpid=%i\n",
           pid_buffer, (int) getpid(), (int) getppid(), openpid);
    return (1);
  }

  return (0);
}

/*----------------------------------------------------------
 check_group_uucp
   accept:     none
   perform:    check if the user is root or in group uucp
   return:     0 on success
   exceptions: none
   comments:
        This checks if the effective user is in group uucp so we can
        create lock files.  If not we give them a warning and bail.
        If its root we just skip the test.
        if someone really wants to override this they can use the			USER_LOCK_DIRECTORY --not recommended.
        In a recent change RedHat 7.2 decided to use group lock.
        In order to get around this we just check the group id
        of the lock directory.
        * Modified to support Debian *
        The problem was that checking the ownership of the lock file
        dir is not enough, in the sense that even if the current user
        is not in the group of the lock directory if the lock
        directory has 777 permissions the lock file can be anyway
        created.  My solution is simply to try to create a tmp file
        there and if it works then we can go on.  Here is my code that
        I tried and seems to work.
        Villa Valerio <valerio.villa@siemens.com>
----------------------------------------------------------*/
int check_group_uucp() {

#ifndef USER_LOCK_DIRECTORY
  FILE *testLockFile ;
  char testLockFileDirName[] = LOCKDIR;
  char testLockFileName[] = "tmpXXXXXX";
  char *testLockAbsFileName;

  testLockAbsFileName = calloc(strlen(testLockFileDirName)
                               + strlen(testLockFileName) + 2, sizeof(char));

  if (NULL == testLockAbsFileName) {
    printf("check_group_uucp(): Insufficient memory");
    return 1;
  }

  strcat(testLockAbsFileName, testLockFileDirName);
  strcat(testLockAbsFileName, "/");
  strcat(testLockAbsFileName, testLockFileName);

  if (-1 == mkstemp(testLockAbsFileName)) {
    free(testLockAbsFileName);
    printf("check_group_uucp(): mktemp malformed string - \
            should not happen");

    return 1;
  }

  testLockFile = fopen(testLockAbsFileName, "w+");

  if (NULL == testLockFile) {
    printf("check_group_uucp(): error testing lock file "
           "creation Error details:");
    printf("%s\n", strerror(errno));
    free(testLockAbsFileName);
    return 1;
  }

  fclose(testLockFile);
  unlink(testLockAbsFileName);
  free(testLockAbsFileName);

#endif /* USER_LOCK_DIRECTORY */
  return 0;
}

#ifdef USE_OLD_CHECK_GROUP_UUCP
int check_group_uucp() {
#ifndef USER_LOCK_DIRECTORY
  int group_count;
  struct passwd *user = getpwuid(geteuid());
  struct stat buf;
  char msg[80];
  gid_t list[ NGROUPS_MAX ];

  if (stat(LOCKDIR, &buf)) {
    sprintf(msg, "check_group_uucp:  Can not find Lock Directory: %s\n", LOCKDIR);
    printf(msg);
    return (1);
  }

  group_count = getgroups(NGROUPS_MAX, list);
  list[ group_count ] = geteuid();

  /* JJO changes start */
  if (user == NULL) {
    printf("Not able to get user groups.\n");
    return 1;
  } else

    /* JJO changes stop */


    if (user->pw_gid) {
      while (group_count >= 0 && buf.st_gid != list[ group_count ]) {
        group_count--;
      }

      if (buf.st_gid == list[ group_count ]) {
        return 0;
      }

      sprintf(msg, "%i %i\n", buf.st_gid, list[ group_count ]);
      printf(msg);
      printf(UUCP_ERROR);
      return 1;
    }

  return 0;
  /*
      if( strcmp( user->pw_name, "root" ) )
      {
          while( *g->gr_mem )
          {
              if( !strcmp( *g->gr_mem, user->pw_name ) )
              {
                  break;
              }
              (void) *g->gr_mem++;
          }
          if( !*g->gr_mem )
          {
              printf( UUCP_ERROR );
              return 1;
          }
      }
  */
#endif /* USER_LOCK_DIRECTORY */
  return 0;
}
#endif /* USE_OLD_CHECK_GROUP_UUCP */


/*----------------------------------------------------------
 The following should be able to follow symbolic links.  I think the stat
 method used below will work on more systems.  This was found while looking
 for information.
 * realpath() doesn't exist on all of the systems my code has to run
   on (HP-UX 9.x, specifically)
----------------------------------------------------------
int different_from_LOCKDIR(const char* ld)
{
    char real_ld[MAXPATHLEN];
    char real_LOCKDIR[MAXPATHLEN];
    if (strncmp(ld, LOCKDIR, strlen(ld)) == 0)
        return 0;
    if (realpath(ld, real_ld) == NULL)
        return 1;
    if (realpath(LOCKDIR, real_LOCKDIR) == NULL)
        return 1;
    if (strncmp(real_ld, real_LOCKDIR, strlen(real_ld)) == 0)
        return 0;
    else
        return 1;
}
*/

/*----------------------------------------------------------
 is_device_locked
   accept:      char * filename.  The device in question including the path.
   perform:     see if one of the many possible lock files is aready there
        if there is a stale lock, remove it.
   return:      1 if the device is locked or somethings wrong.
        0 if its possible to create our own lock file.
   exceptions:  none
   comments:    check if the device is already locked
----------------------------------------------------------*/
int is_device_locked(const char *port_filename) {
  const char *lockdirs[] = { "/etc/locks", "/usr/spool/kermit",
                             "/usr/spool/locks", "/usr/spool/uucp", "/usr/spool/uucp/",
                             "/usr/spool/uucp/LCK", "/var/lock", "/var/lock/modem",
                             "/var/spool/lock", "/var/spool/locks", "/var/spool/uucp",
                             LOCKDIR, NULL
                           };
  const char *lockprefixes[] = { "LCK..", "lk..", "LK.", NULL };
  char *p, file[80], pid_buffer[20];
  int i = 0, j, k, fd, pid;
  struct stat buf, buf2, lockbuf;

  j = strlen(port_filename);
  p = (char *) port_filename + j;

  while (*(p - 1) != '/' && j-- != 1) {
    p--;
  }

  stat(LOCKDIR, &lockbuf);

  while (lockdirs[i]) {
    /*
       Look for lockfiles in all known places other than the
       defined lock directory for this system
       report any unexpected lockfiles.
       Is the suspect lockdir there?
       if it is there is it not the expected lock dir?
    */
    if (!stat(lockdirs[i], &buf2) &&
        buf2.st_ino != lockbuf.st_ino &&
        strncmp(lockdirs[i], LOCKDIR, strlen(lockdirs[i]))) {
      j = strlen(port_filename);
      p = (char *) port_filename + j;

      /*
         SCO Unix use lowercase all the time
          taj
      */
      while (*(p - 1) != '/' && j-- != 1) {
#if defined ( __unixware__ )
        *p = tolower(*p);
#endif /* __unixware__ */
        p--;
      }

      k = 0;

      while (lockprefixes[k]) {
        /* FHS style */
        sprintf(file, "%s/%s%s", lockdirs[i],
                lockprefixes[k], p);

        if (stat(file, &buf) == 0) {
//                    sprintf( message, UNEXPECTED_LOCK_FILE,
//                        file );
//                    printf( message );
          return 1;
        }

        /* UUCP style */
        stat(port_filename, &buf);
        sprintf(file, "%s/%s%03d.%03d.%03d",
                lockdirs[i],
                lockprefixes[k],
                (int) major(buf.st_dev),
                (int) major(buf.st_rdev),
                (int) minor(buf.st_rdev)
               );

        if (stat(file, &buf) == 0) {
//                    sprintf( message, UNEXPECTED_LOCK_FILE,
//                        file );
//                    printf( message );
          return 1;
        }

        k++;
      }
    }

    i++;
  }

  /*
      OK.  We think there are no unexpect lock files for this device
      Lets see if there any stale lock files that need to be
      removed.
  */

#ifdef FHS
  /*  FHS standard locks */
  i = strlen(port_filename);
  p = (char *) port_filename + i;

  while (*(p - 1) != '/' && i-- != 1) {
#if defined ( __unixware__ )
    *p = tolower(*p);
#endif /* __unixware__ */
    p--;
  }

  sprintf(file, "%s/%s%s", LOCKDIR, LOCKFILEPREFIX, p);
#else

  /*  UUCP standard locks */
  if (stat(port_filename, &buf) != 0) {
    printf("RXTX is_device_locked() could not find device.\n");
    return 1;
  }

  sprintf(file, "%s/LK.%03d.%03d.%03d",
          LOCKDIR,
          (int) major(buf.st_dev),
          (int) major(buf.st_rdev),
          (int) minor(buf.st_rdev)
         );

#endif /* FHS */

  if (stat(file, &buf) == 0) {

    /* check if its a stale lock */
    fd = open(file, O_RDONLY);
    read(fd, pid_buffer, 11);
    /* FIXME null terminiate pid_buffer? need to check in Solaris */
    close(fd);
    sscanf(pid_buffer, "%d", &pid);

    if (kill((pid_t) pid, 0) && errno == ESRCH) {
      printf(
        "RXTX Warning:  Removing stale lock file. %s\n",
        file);

      if (unlink(file) != 0) {
        printf("RXTX Error:  Unable to \
                    remove stale lock file: %s\n",
               file
              );
        return 1;
      }
    }
  }

  return 0;
}
#endif /* WIN32 */

