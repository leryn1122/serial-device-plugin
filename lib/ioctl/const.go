package ioctl

import "syscall"

// noinspection GoSnakeCaseUsage
const (
	O_ACCMODE   = syscall.O_ACCMODE
	O_APPEND    = syscall.O_APPEND
	O_ASYNC     = syscall.O_ASYNC
	O_CLOEXEC   = syscall.O_CLOEXEC
	O_CREAT     = syscall.O_CREAT
	O_DIRECT    = syscall.O_DIRECT
	O_DIRECTORY = syscall.O_DIRECTORY
	O_DSYNC     = syscall.O_DSYNC
	O_EXCL      = syscall.O_EXCL
	O_FSYNC     = syscall.O_FSYNC
	O_LARGEFILE = syscall.O_LARGEFILE
	O_NDELAY    = syscall.O_NDELAY
	O_NOATIME   = syscall.O_NOATIME
	O_NOCTTY    = syscall.O_NOCTTY
	O_NOFOLLOW  = syscall.O_NOFOLLOW
	O_NONBLOCK  = syscall.O_NONBLOCK
	O_RDONLY    = syscall.O_RDONLY
	O_RDWR      = syscall.O_RDWR
	O_RSYNC     = syscall.O_RSYNC
	O_SYNC      = syscall.O_SYNC
	O_TRUNC     = syscall.O_TRUNC
	O_WRONLY    = syscall.O_WRONLY
)
