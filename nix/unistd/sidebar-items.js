initSidebarItems({"constant":[["ROOT","Constant for UID = 0"]],"enum":[["FchownatFlags","Flags for `fchownat` function."],["ForkResult","Represents the successful result of calling `fork`"],["PathconfVar","Variable names for `pathconf`"],["SysconfVar","Variable names for `sysconf`"],["UnlinkatFlags","Flags for `unlinkat` function."],["Whence","Directive that tells [`lseek`] and [`lseek64`] what the offset is relative to."]],"fn":[["access","Checks the file named by `path` for accessibility according to the flags given by `amode` See access(2)"],["chdir","Change the current working directory of the calling process (see chdir(2))."],["chown","Change the ownership of the file at `path` to be owned by the specified `owner` (user) and `group` (see chown(2))."],["chroot",""],["close","Close a raw file descriptor"],["daemon","Daemonize this process by detaching from the controlling terminal (see daemon(3))."],["dup","Create a copy of the specified file descriptor (see dup(2))."],["dup2","Create a copy of the specified file descriptor using the specified fd (see dup(2))."],["dup3","Create a new copy of the specified file descriptor using the specified fd and flags (see dup(2))."],["execv","Replace the current process image with a new one (see exec(3))."],["execve","Replace the current process image with a new one (see execve(2))."],["execveat","Execute program relative to a directory file descriptor (see execveat(2))."],["execvp","Replace the current process image with a new one and replicate shell `PATH` searching behavior (see exec(3))."],["execvpe","Replace the current process image with a new one and replicate shell `PATH` searching behavior (see `execvpe(3)`)."],["fchdir","Change the current working directory of the process to the one given as an open file descriptor (see fchdir(2))."],["fchownat","Change the ownership of the file at `path` to be owned by the specified `owner` (user) and `group`."],["fdatasync","Synchronize the data of a file"],["fexecve","Replace the current process image with a new one (see fexecve(2))."],["fork","Create a new child process duplicating the parent process (see fork(2))."],["fpathconf","Like `pathconf`, but works with file descriptors instead of paths (see fpathconf(2))"],["fsync","Synchronize changes to a file"],["ftruncate","Truncate a file to a specified length"],["getcwd","Returns the current directory as a `PathBuf`"],["getegid","Get the effective group ID"],["geteuid","Get the effective user ID"],["getgid","Get the real group ID"],["getgrouplist","Calculate the supplementary group access list."],["getgroups","Get the list of supplementary group IDs of the calling process."],["gethostname","Get the host name and store it in the provided buffer, returning a pointer the `CStr` in that buffer on success (see gethostname(2))."],["getpgid",""],["getpgrp","Get the group id of the calling process (see getpgrp(3))."],["getpid","Get the pid of this process (see getpid(2))."],["getppid","Get the pid of this processes' parent (see getpid(2))."],["getsid","Get the process group ID of a session leader getsid(2)."],["gettid","Get the caller's thread ID (see gettid(2)."],["getuid","Get a real user ID"],["initgroups","Initialize the supplementary group access list."],["isatty",""],["lseek","Move the read/write file offset."],["lseek64",""],["mkdir","Creates new directory `path` with access rights `mode`.  (see mkdir(2))"],["mkfifo","Creates new fifo special file (named pipe) with path `path` and access rights `mode`."],["mkstemp","Creates a regular file which persists even after process termination"],["pathconf","Get path-dependent configurable system variables (see pathconf(2))"],["pause","Suspend the thread until a signal is received."],["pipe","Create an interprocess channel."],["pipe2","Like `pipe`, but allows setting certain file descriptor flags."],["pivot_root",""],["read","Read from a raw file descriptor."],["setegid","Set the effective group ID"],["seteuid","Set the effective user ID"],["setgid","Set the group ID"],["setgroups","Set the list of supplementary group IDs for the calling process."],["sethostname","Set the system host name (see sethostname(2))."],["setpgid","Set a process group ID (see setpgid(2))."],["setresgid","Sets the real, effective, and saved gid. (see setresuid(2))"],["setresuid","Sets the real, effective, and saved uid. (see setresuid(2))"],["setsid","Create new session and set process group id (see setsid(2))."],["setuid","Set the user ID"],["sleep","Suspend execution for an interval of time"],["symlinkat","Creates a symbolic link at `path2` which points to `path1`."],["sync","Commit filesystem caches to disk"],["sysconf","Get configurable system variables (see sysconf(3))"],["tcgetpgrp","Get the terminal foreground process group (see tcgetpgrp(3))."],["tcsetpgrp","Set the terminal foreground process group (see tcgetpgrp(3))."],["truncate","Truncate a file to a specified length"],["unlink","Remove a directory entry"],["unlinkat","Remove a directory entry"],["write","Write to a raw file descriptor."]],"mod":[["acct",""],["alarm","Alarm signal scheduling."]],"struct":[["AccessFlags","Options for access()"],["Gid","Group identifier"],["Pid","Process identifier"],["Uid","User identifier"]]});